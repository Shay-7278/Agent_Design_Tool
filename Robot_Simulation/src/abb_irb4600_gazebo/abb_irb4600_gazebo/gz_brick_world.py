"""Blocking entity operations and feedback-confirmed Gazebo detachable joints."""

from __future__ import annotations

import threading
import time
import subprocess
from pathlib import Path
from google.protobuf import text_format

from gz.transport13 import Node
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.entity_plugin_v_pb2 import EntityPlugin_V
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.pose_pb2 import Pose as GazeboPose
from gz.msgs10.scene_pb2 import Scene
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.serialized_map_pb2 import SerializedStepMap

from .motion import Pose


class GazeboCommandError(RuntimeError):
    pass


def _component_id(name: str) -> int:
    """Gazebo Common's FNV-1a hash64, used by the component factory."""
    value = 0xcbf29ce484222325
    for byte in name.encode('utf-8'):
        value = ((value ^ byte) * 0x100000001b3) & 0xffffffffffffffff
    return value


DETACHABLE_JOINT_TYPE = _component_id('gz_sim_components.DetachableJoint')


def detachable_joints(snapshot):
    """Read actual ECM components, not one-shot transport notifications."""
    joints = {}
    for entity_id, entity in snapshot.state.entities.items():
        for component in entity.components.values():
            if component.type != DETACHABLE_JOINT_TYPE:
                continue
            try:
                parent, child, kind = component.component.decode('utf-8').split()
                joints[entity_id] = (int(parent), int(child), kind)
            except (ValueError, UnicodeError) as exc:
                raise GazeboCommandError('invalid Gazebo detachable-joint component') from exc
    return joints


class GazeboBrickWorld:
    def __init__(
        self,
        world_name: str,
        brick_sdf: str | Path,
        parent_model: str = "abb_irb4600_workcell",
        parent_link: str = "LL_link_6",
    ) -> None:
        self.world_name = world_name
        self.brick_sdf = str(Path(brick_sdf).resolve())
        self.parent_model = parent_model
        self.parent_link = parent_link
        self.node = Node()
        self._poses = {}
        self._pose_sequence = {}
        self._pose_time = {}
        self._states = {}
        self._notifications = {}
        self._plugins = set()
        self._publishers = {}
        self._link_ids = {}
        self.attachment_checks = {}
        self._sim_time = None
        self._clock_progress_wall = time.monotonic()
        self._condition = threading.Condition()
        self.node.subscribe(Pose_V, f"/world/{world_name}/pose/info", self._on_poses)
        self.node.subscribe(Clock, f"/world/{world_name}/clock", self._on_clock)

    def _on_clock(self, message):
        with self._condition:
            stamp = message.sim.sec + message.sim.nsec * 1e-9
            if self._sim_time is None or stamp != self._sim_time:
                self._clock_progress_wall = time.monotonic()
            self._sim_time = stamp
            self._condition.notify_all()

    def _on_poses(self, message):
        with self._condition:
            for p in message.pose:
                self._poses[p.name] = Pose((p.position.x, p.position.y, p.position.z),
                                          (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))
                self._pose_sequence[p.name] = self._pose_sequence.get(p.name, 0) + 1
                self._pose_time[p.name] = (message.header.stamp.sec + message.header.stamp.nsec * 1e-9,
                                           time.monotonic())
            self._condition.notify_all()

    def _wait(self, predicate, description, timeout=15.0):
        deadline = time.monotonic() + timeout
        with self._condition:
            while not predicate():
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise GazeboCommandError(f"timed out waiting for {description}")
                self._condition.wait(min(remaining, 0.2))

    def _request(self, service, request, response_type=Boolean):
        try:
            result = subprocess.run([
                "gz", "service", "-s", f"/world/{self.world_name}/{service}",
                "--reqtype", request.DESCRIPTOR.full_name,
                "--reptype", response_type.DESCRIPTOR.full_name,
                "--timeout", "10000", "--req", text_format.MessageToString(request),
            ], capture_output=True, text=True, timeout=15)
        except subprocess.TimeoutExpired as exc:
            raise GazeboCommandError(f'Gazebo {service} RPC did not respond within 15 real seconds') from exc
        reply = response_type()
        if result.returncode or not result.stdout.strip():
            raise GazeboCommandError(f"Gazebo {service}: {result.stderr or result.stdout}")
        text_format.Parse(result.stdout, reply)
        if response_type is Boolean and not reply.data:
            raise GazeboCommandError(f"Gazebo {service} returned false")
        return reply

    def models(self):
        return {model.name: model.id for model in self._request("scene/info", Empty(), Scene).model}

    def _resolve_links(self, name):
        if name not in self._link_ids:
            scene = self._request('scene/info', Empty(), Scene)
            models = {m.name: m for m in scene.model}
            try:
                parent = next(l.id for l in models[self.parent_model].link if l.name == self.parent_link)
                child = next(l.id for l in models[name].link if l.name == 'brick_link')
            except (KeyError, StopIteration) as exc:
                raise GazeboCommandError(f'cannot resolve attachment links for {name}') from exc
            self._link_ids[name] = (models[self.parent_model].id, parent, child)
        return self._link_ids[name]

    def _joint_readback(self, name):
        _, parent, child = self._resolve_links(name)
        snapshot = self._request('state', Empty(), SerializedStepMap)
        if parent not in snapshot.state.entities or child not in snapshot.state.entities:
            raise GazeboCommandError(f'attachment link disappeared for {name}')
        matching = {eid: data for eid, data in detachable_joints(snapshot).items() if data[1] == child}
        if any(data != (parent, child, 'fixed') for data in matching.values()) or len(matching) > 1:
            raise GazeboCommandError(f'unexpected physical joint for {name}: {matching}')
        stamp = snapshot.stats.sim_time.sec + snapshot.stats.sim_time.nsec * 1e-9
        return next(iter(matching), None), stamp

    def _wait_for_joint(self, name, attached, timeout_sim=10.0, stalled_wall=30.0):
        """Confirm ECM state; retry idempotent requests and bound a stalled clock."""
        start_wall = time.monotonic()
        start_sim = previous_sim = self._sim_time
        action = 'attach' if attached else 'detach'
        last_error = None
        while True:
            now = time.monotonic()
            sim_now = self._sim_time
            if sim_now is not None:
                if start_sim is None:
                    start_sim = sim_now
                if previous_sim is not None and sim_now < previous_sim - 0.001:
                    raise GazeboCommandError(f'Gazebo clock reset during {action} of {name}')
                previous_sim = sim_now
                if sim_now - start_sim > timeout_sim:
                    raise GazeboCommandError(f'{action} of {name} unconfirmed after {timeout_sim} simulation seconds; last readback={last_error}')
            if now - max(start_wall, self._clock_progress_wall) > stalled_wall:
                raise GazeboCommandError(f'Gazebo clock unavailable or stopped for {stalled_wall} real seconds during {action} of {name}; last readback={last_error}')
            try:
                entity_id, stamp = self._joint_readback(name)
                if (entity_id is not None) == attached:
                    state = 'attached' if attached else 'detached'
                    self._states[name] = state
                    self.attachment_checks[name] = {
                        'state': state, 'joint_entity': entity_id,
                        'parent_link': self._link_ids[name][1], 'child_link': self._link_ids[name][2],
                        'simulation_stamp': stamp, 'verification': 'Gazebo ECM readback',
                        'notification': self._notifications.get(name),
                    }
                    return
                last_error = f'joint_entity={entity_id}'
            except GazeboCommandError as exc:
                last_error = str(exc)
            # Native requests are idempotent. Do not reload a second plugin.
            self._publishers[name, action].publish(Empty())
            with self._condition:
                self._condition.wait(0.1)

    def pose(self, name):
        return self.observation(name)[0]

    def observation(self, name, after_sequence=None):
        self._wait(lambda: name in self._poses
                   and time.monotonic() - self._pose_time[name][1] < 5.0
                   and (after_sequence is None or self._pose_sequence[name] > after_sequence),
                   f"fresh pose of {name}")
        with self._condition:
            return self._poses[name], self._pose_sequence[name], self._pose_time[name][0]

    def spawn(self, name: str, pose: Pose) -> None:
        x, y, z = pose.position
        qx, qy, qz, qw = pose.orientation_xyzw
        request = EntityFactory(sdf_filename=self.brick_sdf, name=name, allow_renaming=False)
        request.pose.position.x, request.pose.position.y, request.pose.position.z = x, y, z
        request.pose.orientation.x, request.pose.orientation.y = qx, qy
        request.pose.orientation.z, request.pose.orientation.w = qz, qw
        self._request("create", request)
        self.pose(name)

    def set_pose(self, name: str, pose: Pose) -> None:
        """Relocate an existing model without changing its collisions or dynamics."""
        request = GazeboPose(name=name)
        request.position.x, request.position.y, request.position.z = pose.position
        (request.orientation.x, request.orientation.y,
         request.orientation.z, request.orientation.w) = pose.orientation_xyzw
        self._request("set_pose", request)

    def attach(self, name: str) -> None:
        topic = f"/assembly/{name}"
        if name not in self._plugins:
            def callback(message):
                with self._condition:
                    self._notifications[name] = message.data
                    self._condition.notify_all()
            self.node.subscribe(StringMsg, topic + "/state", callback)
            for action in ("attach", "detach"):
                self._publishers[name, action] = self.node.advertise(topic + "/" + action, Empty)
            request = EntityPlugin_V()
            request.entity.id = self._resolve_links(name)[0]
            request.entity.type = Entity.MODEL
            plugin = request.plugins.add()
            plugin.filename = "gz-sim-detachable-joint-system"
            plugin.name = "gz::sim::systems::DetachableJoint"
            plugin.innerxml = (f"<parent_link>{self.parent_link}</parent_link>"
                               f"<child_model>{name}</child_model><child_link>brick_link</child_link>"
                               f"<attach_topic>{topic}/attach</attach_topic>"
                               f"<detach_topic>{topic}/detach</detach_topic>"
                               f"<output_topic>{topic}/state</output_topic>")
            # Loading at grasp time intentionally creates the initial attachment.
            self._request("entity/system/add", request)
            self._plugins.add(name)
        self._wait_for_joint(name, attached=True)

    def detach(self, name: str) -> None:
        if name not in self._plugins:
            raise GazeboCommandError(f"no attachment registered for {name}")
        self._wait_for_joint(name, attached=False)

    def delete(self, name: str) -> None:
        if self._states.get(name) == "attached":
            self.detach(name)
        self._request("remove/blocking", Entity(name=name, type=Entity.MODEL))
