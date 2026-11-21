"""Sequential binary RPC over one long-lived native Gazebo transport node."""
from __future__ import annotations

import os
import select
import struct
import subprocess
import tempfile
import threading
import time
from pathlib import Path


class ServiceClientError(RuntimeError):
    pass


class ServiceTimeout(ServiceClientError):
    """No reply was received; the mutation may nevertheless have succeeded."""


class ServiceRejected(ServiceClientError):
    pass


class PersistentServiceClient:
    def __init__(self, binary=None):
        if binary is None:
            binary = os.environ.get('GAZEBO_SERVICE_CLIENT_BINARY')
        if binary is None:
            from ament_index_python.packages import get_package_prefix
            binary = Path(get_package_prefix('abb_irb4600_gazebo')) / 'lib/abb_irb4600_gazebo/gazebo_service_client'
        self._lock = threading.Lock()
        self._broken = False
        self._stderr = tempfile.TemporaryFile()
        try:
            self._process = subprocess.Popen([str(binary)], stdin=subprocess.PIPE,
                                             stdout=subprocess.PIPE, stderr=self._stderr)
        except OSError as exc:
            self._stderr.close()
            raise ServiceClientError(f'cannot start persistent Gazebo client: {exc}') from exc

    @property
    def pid(self):
        return self._process.pid

    @property
    def healthy(self):
        return not self._broken and self._process.poll() is None

    def _read_exact(self, count, deadline):
        chunks = []
        while count:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([self._process.stdout], [], [], remaining)[0]:
                raise ServiceTimeout('persistent Gazebo client did not return a complete reply')
            chunk = os.read(self._process.stdout.fileno(), count)
            if not chunk:
                raise ServiceClientError(f'persistent Gazebo client exited: {self._process.poll()}')
            chunks.append(chunk)
            count -= len(chunk)
        return b''.join(chunks)

    def request(self, service, request, response_type, timeout_ms=10000):
        timeout_ms = int(timeout_ms)
        if timeout_ms <= 0:
            raise ValueError('service timeout must be positive')
        fields = (service.encode(), request.DESCRIPTOR.full_name.encode(),
                  response_type.DESCRIPTOR.full_name.encode(), request.SerializeToString())
        if any(len(field) > 64 * 1024 * 1024 for field in fields):
            raise ServiceClientError('service request exceeds 64 MiB field limit')
        frame = struct.pack('!5I', timeout_ms, *(len(field) for field in fields)) + b''.join(fields)
        with self._lock:
            if self._broken or self._process.poll() is not None:
                raise ServiceClientError('persistent Gazebo client is not running')
            try:
                self._process.stdin.write(frame)
                self._process.stdin.flush()
                deadline = time.monotonic() + timeout_ms / 1000 + 2.0
                status, length = struct.unpack('!2I', self._read_exact(8, deadline))
                if length > 64 * 1024 * 1024:
                    raise ServiceClientError('invalid service response length')
                payload = self._read_exact(length, deadline)
            except (ServiceClientError, BrokenPipeError, OSError) as exc:
                # A partial/late response cannot be associated with a later
                # request. Poison this worker rather than reuse its byte stream.
                self._broken = True
                if self._process.poll() is None:
                    self._process.terminate()
                raise ServiceTimeout(f'{service}: persistent client stream lost; outcome unknown: {exc}') from exc
            if status == 1:
                raise ServiceTimeout(f'{service}: {payload.decode(errors="replace")}')
            if status == 2:
                raise ServiceRejected(f'{service}: {payload.decode(errors="replace")}')
            if status != 0:
                raise ServiceClientError(f'{service}: {payload.decode(errors="replace")}')
            response = response_type()
            try:
                response.ParseFromString(payload)
            except Exception as exc:
                self._broken = True
                if self._process.poll() is None:
                    self._process.terminate()
                raise ServiceClientError(f'{service}: invalid protobuf response') from exc
            return response

    def close(self):
        with self._lock:
            process = self._process
            if process.poll() is None:
                process.stdin.close()
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    try:
                        process.wait(timeout=1)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=1)
            if not process.stdin.closed:
                process.stdin.close()
            process.stdout.close()
            self._stderr.close()

