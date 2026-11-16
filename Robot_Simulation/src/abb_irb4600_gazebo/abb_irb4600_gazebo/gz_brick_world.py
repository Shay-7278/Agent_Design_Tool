"""Gazebo CLI adapter for dynamic brick spawn, attach, detach, and cleanup.

The world already loads Gazebo's UserCommands system, so fixed joint entities
can be created and removed at runtime.  This avoids hard-coding a finite list
of DetachableJoint child model names in the robot SDF.
"""

from __future__ import annotations

import subprocess
from pathlib import Path

from .motion import Pose


class GazeboCommandError(RuntimeError):
    pass


class GazeboBrickWorld:
    def __init__(
        self,
        world_name: str,
        brick_sdf: str | Path,
        parent_model: str = "abb_irb4600_workcell",
        parent_link: str = "LL_schunk_tcp",
    ) -> None:
        self.world_name = world_name
        self.brick_sdf = str(Path(brick_sdf).resolve())
        self.parent_model = parent_model
        self.parent_link = parent_link

    def _service(self, service: str, request_type: str, response_type: str, request: str) -> None:
        command = [
            "gz", "service", "-s", f"/world/{self.world_name}/{service}",
            "--reqtype", request_type, "--reptype", response_type,
            "--timeout", "2000", "--req", request,
        ]
        result = subprocess.run(command, text=True, capture_output=True)
        if result.returncode != 0 or "data: true" not in result.stdout.lower():
            raise GazeboCommandError(
                f"Gazebo {service} failed ({result.returncode}): "
                f"{result.stderr.strip() or result.stdout.strip()}"
            )

    def spawn(self, name: str, pose: Pose) -> None:
        x, y, z = pose.position
        qx, qy, qz, qw = pose.orientation_xyzw
        request = (
            f'sdf_filename: "{self.brick_sdf}" name: "{name}" '
            f'pose: {{position: {{x: {x} y: {y} z: {z}}} '
            f'orientation: {{x: {qx} y: {qy} z: {qz} w: {qw}}}}}'
        )
        self._service("create", "gz.msgs.EntityFactory", "gz.msgs.Boolean", request)

    def attach(self, name: str) -> None:
        joint_name = self._joint_name(name)
        sdf = (
            "<sdf version='1.9'><joint name='{joint}' type='fixed'>"
            "<parent>{parent}::{parent_link}</parent>"
            "<child>{child}::brick_link</child></joint></sdf>"
        ).format(
            joint=joint_name,
            parent=self.parent_model,
            parent_link=self.parent_link,
            child=name,
        )
        request = f'sdf: "{sdf.replace(chr(34), chr(92) + chr(34))}"'
        self._service("create", "gz.msgs.EntityFactory", "gz.msgs.Boolean", request)

    def detach(self, name: str) -> None:
        request = f'name: "{self._joint_name(name)}" type: JOINT'
        self._service("remove", "gz.msgs.Entity", "gz.msgs.Boolean", request)

    def delete(self, name: str) -> None:
        # Remove the joint first if a failed run left it behind.
        try:
            self.detach(name)
        except GazeboCommandError:
            pass
        request = f'name: "{name}" type: MODEL'
        self._service("remove", "gz.msgs.Entity", "gz.msgs.Boolean", request)

    @staticmethod
    def _joint_name(name: str) -> str:
        return f"{name}__ll_grasp_fixed_joint"
