#!/usr/bin/env python3
"""Exercise each dual-IRB4600 controlled joint with a small reversible motion."""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


LEFT_JOINTS = ["joint_LL_ext_1", *[f"joint_LL_{index}" for index in range(1, 7)]]
RIGHT_JOINTS = ["joint_RR_ext_1", *[f"joint_RR_{index}" for index in range(1, 7)]]
ALL_JOINTS = LEFT_JOINTS + RIGHT_JOINTS

TEST_DISPLACEMENT = 0.02  # metres for tracks; radians for revolute joints
POSITION_TOLERANCE = 0.005
OTHER_SIDE_TOLERANCE = 0.005
STOPPED_VELOCITY_TOLERANCE = 0.01


class ProvisionalMotionValidator(Node):
    def __init__(self) -> None:
        super().__init__("validate_provisional_motion")
        self._positions: dict[str, float] = {}
        self._velocities: dict[str, float] = {}
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self._action_clients = {
            "LL": ActionClient(
                self,
                FollowJointTrajectory,
                "/LL_manipulator_controller/follow_joint_trajectory",
            ),
            "RR": ActionClient(
                self,
                FollowJointTrajectory,
                "/RR_manipulator_controller/follow_joint_trajectory",
            ),
        }

    def _on_joint_state(self, message: JointState) -> None:
        for index, joint_name in enumerate(message.name):
            if index < len(message.position):
                self._positions[joint_name] = message.position[index]
            if index < len(message.velocity):
                self._velocities[joint_name] = message.velocity[index]

    def wait_for_interfaces(self) -> None:
        deadline = time.monotonic() + 20.0
        while not set(ALL_JOINTS).issubset(self._positions):
            if time.monotonic() >= deadline:
                missing = sorted(set(ALL_JOINTS) - set(self._positions))
                raise RuntimeError(f"Timed out waiting for joint states: {missing}")
            rclpy.spin_once(self, timeout_sec=0.1)

        for side, client in self._action_clients.items():
            if not client.wait_for_server(timeout_sec=20.0):
                raise RuntimeError(f"{side} trajectory action server is unavailable")

    def send_positions(self, side: str, joint_names: list[str], positions: list[float]) -> None:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(joint_names)
        point.time_from_start = Duration(nanosec=500_000_000)
        goal.trajectory.points = [point]

        goal_future = self._action_clients[side].send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future, timeout_sec=10.0)
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"{side} controller rejected the trajectory")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=20.0)
        wrapped_result = result_future.result()
        if wrapped_result is None:
            raise RuntimeError(f"{side} controller did not return a trajectory result")
        result = wrapped_result.result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(
                f"{side} trajectory failed with code {result.error_code}: "
                f"{result.error_string}"
            )

        rclpy.spin_once(self, timeout_sec=0.2)

    def wait_for_position(self, joint_name: str, target: float) -> float:
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            position = self._positions[joint_name]
            if math.isclose(position, target, abs_tol=POSITION_TOLERANCE):
                return position
        raise RuntimeError(
            f"{joint_name} did not converge to {target}; "
            f"last position was {self._positions[joint_name]}"
        )

    def validate_joint(self, side: str, joint_names: list[str], test_joint: str) -> None:
        other_side = RIGHT_JOINTS if side == "LL" else LEFT_JOINTS
        other_before = {name: self._positions[name] for name in other_side}

        target_positions = [
            TEST_DISPLACEMENT if name == test_joint else 0.0 for name in joint_names
        ]
        self.send_positions(side, joint_names, target_positions)

        achieved = self.wait_for_position(test_joint, TEST_DISPLACEMENT)
        if achieved <= 0.0:
            raise RuntimeError(
                f"{test_joint} moved in the wrong direction or did not move: {achieved}"
            )
        if not math.isclose(
            achieved, TEST_DISPLACEMENT, abs_tol=POSITION_TOLERANCE
        ):
            raise RuntimeError(
                f"{test_joint} reached {achieved}, expected {TEST_DISPLACEMENT}"
            )

        for other_joint in other_side:
            movement = abs(self._positions[other_joint] - other_before[other_joint])
            if movement > OTHER_SIDE_TOLERANCE:
                raise RuntimeError(
                    f"{other_joint} moved {movement} while testing {test_joint}"
                )

        self.send_positions(side, joint_names, [0.0] * len(joint_names))
        returned = self.wait_for_position(test_joint, 0.0)
        if not math.isclose(returned, 0.0, abs_tol=POSITION_TOLERANCE):
            raise RuntimeError(f"{test_joint} failed to return to zero: {returned}")

        print(
            f"PASS {test_joint}: reached {achieved:.6f}, returned {returned:.6f}, "
            "other controller remained stationary"
        )

    def validate_stopped_state(self) -> None:
        deadline = time.monotonic() + 10.0
        unstable: dict[str, float] = {}
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            unstable = {
                name: velocity
                for name, velocity in self._velocities.items()
                if name in ALL_JOINTS
                and abs(velocity) > STOPPED_VELOCITY_TOLERANCE
            }
            if not unstable:
                return
        if unstable:
            raise RuntimeError(f"Joints exceeded stopped velocity tolerance: {unstable}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--scope",
        choices=("all", "arms", "tracks"),
        default="all",
        help="Select all joints, only the 12 arm joints, or only the two tracks",
    )
    arguments = parser.parse_args()

    rclpy.init()
    validator = ProvisionalMotionValidator()
    try:
        validator.wait_for_interfaces()
        if arguments.scope == "arms":
            left_test_joints = LEFT_JOINTS[1:]
            right_test_joints = RIGHT_JOINTS[1:]
        elif arguments.scope == "tracks":
            left_test_joints = LEFT_JOINTS[:1]
            right_test_joints = RIGHT_JOINTS[:1]
        else:
            left_test_joints = LEFT_JOINTS
            right_test_joints = RIGHT_JOINTS

        for joint_name in left_test_joints:
            validator.validate_joint("LL", LEFT_JOINTS, joint_name)
        for joint_name in right_test_joints:
            validator.validate_joint("RR", RIGHT_JOINTS, joint_name)
        validator.validate_stopped_state()
        tested_joint_count = len(left_test_joints) + len(right_test_joints)
        print(
            f"PASS: {tested_joint_count} {arguments.scope} joints completed "
            "provisional motion and stability checks"
        )
    finally:
        validator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
