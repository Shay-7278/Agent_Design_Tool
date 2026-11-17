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
from gz.msgs10.scene_pb2 import Scene
from gz.msgs10.stringmsg_pb2 import StringMsg

from .motion import Pose


class GazeboCommandError(RuntimeError):
    pass


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
        self._plugins = set()
        self._publishers = {}
        self._condition = threading.Condition()
        self.node.subscribe(Pose_V, f"/world/{world_name}/pose/info", self._on_poses)

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
        result = subprocess.run([
            "gz", "service", "-s", f"/world/{self.world_name}/{service}",
            "--reqtype", request.DESCRIPTOR.full_name,
            "--reptype", response_type.DESCRIPTOR.full_name,
            "--timeout", "10000", "--req", text_format.MessageToString(request),
        ], capture_output=True, text=True, timeout=15)
        reply = response_type()
        if result.returncode or not result.stdout.strip():
            raise GazeboCommandError(f"Gazebo {service}: {result.stderr or result.stdout}")
        text_format.Parse(result.stdout, reply)
        if response_type is Boolean and not reply.data:
            raise GazeboCommandError(f"Gazebo {service} returned false")
        return reply

    def models(self):
        return {model.name: model.id for model in self._request("scene/info", Empty(), Scene).model}

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

    def attach(self, name: str) -> None:
        topic = f"/assembly/{name}"
        if name not in self._plugins:
            def callback(message):
                with self._condition:
                    self._states[name] = message.data
                    self._condition.notify_all()
            self.node.subscribe(StringMsg, topic + "/state", callback)
            for action in ("attach", "detach"):
                self._publishers[name, action] = self.node.advertise(topic + "/" + action, Empty)
            request = EntityPlugin_V()
            request.entity.id = self.models()[self.parent_model]
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
        else:
            self._publishers[name, "attach"].publish(Empty())
        self._wait(lambda: self._states.get(name) == "attached", f"attachment of {name}")

    def detach(self, name: str) -> None:
        if name not in self._plugins:
            raise GazeboCommandError(f"no attachment registered for {name}")
        self._publishers[name, "detach"].publish(Empty())
        self._wait(lambda: self._states.get(name) == "detached", f"release of {name}")

    def delete(self, name: str) -> None:
        if self._states.get(name) == "attached":
            self.detach(name)
        self._request("remove/blocking", Entity(name=name, type=Entity.MODEL))
