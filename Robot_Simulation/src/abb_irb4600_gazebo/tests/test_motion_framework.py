#!/usr/bin/env python3
"""Unit tests for the TCP-independent Grasshopper motion contracts."""

import unittest

from abb_irb4600_gazebo.motion import (
    BrickMotionTargets,
    MotionKind,
    Pose,
    build_pick_place_steps,
    classify_reachability,
    rail_position,
)


IDENTITY = (0.0, 0.0, 0.0, 1.0)


class MotionFrameworkTests(unittest.TestCase):
    def setUp(self) -> None:
        self.targets = BrickMotionTargets(
            "brick_001",
            Pose((1.0, 2.0, 0.1), IDENTITY),
            Pose((3.0, 4.0, 0.2), IDENTITY),
        )

    def test_sequence_matches_recovered_grasshopper_flow(self) -> None:
        steps = build_pick_place_steps(self.targets, retract_m=0.25)
        self.assertEqual(
            [step.kind for step in steps],
            [
                MotionKind.MOVE_J,
                MotionKind.MOVE_L,
                MotionKind.WAIT,
                MotionKind.GRIPPER_CLOSE,
                MotionKind.WAIT,
                MotionKind.MOVE_L,
                MotionKind.MOVE_J,
                MotionKind.MOVE_L,
                MotionKind.WAIT,
                MotionKind.GRIPPER_OPEN,
                MotionKind.WAIT,
                MotionKind.MOVE_L,
            ],
        )
        self.assertEqual(steps[0].target.position, (1.0, 2.0, 0.35))
        self.assertEqual(steps[-1].target.position, (3.0, 4.0, 0.45))

    def test_track_projection_is_si_and_clamped(self) -> None:
        self.assertEqual(
            rail_position(self.targets.pickup, (0, 0, 0), (0, 1, 0)), 2.0
        )
        self.assertEqual(
            rail_position(self.targets.placement, (0, 0, 0), (0, 1, 0)), 4.0
        )

    def test_reachability_is_separate_from_motion_generation(self) -> None:
        def below_limit(pose: Pose) -> tuple[bool, str | None]:
            valid = pose.position[0] < 2.0
            return valid, None if valid else "x limit"

        result = classify_reachability([self.targets], below_limit, 0.25)
        self.assertFalse(result[0].reachable)
        self.assertEqual(result[0].reason, "x limit")

    def test_pose_rejects_zero_quaternion(self) -> None:
        with self.assertRaises(ValueError):
            Pose((0, 0, 0), (0, 0, 0, 0))


if __name__ == "__main__":
    unittest.main()
