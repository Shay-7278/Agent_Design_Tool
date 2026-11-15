import unittest

from abb_irb4600_gazebo.motion import MotionKind, MotionStep, Pose
from abb_irb4600_gazebo.target_motion import (
    RX_PI,
    convert_motion_steps_to_ll,
    flange_target_from_tcp,
    tcp_target_from_brick,
)


class TargetMotionTests(unittest.TestCase):
    def test_original_local_half_turn_flips_z(self):
        target = tcp_target_from_brick(Pose((0, 0, 0), (0, 0, 0, 1)))
        self.assertEqual(target.orientation_xyzw, RX_PI)

    def test_identity_tool_transform_preserves_tcp_position(self):
        tcp = Pose((1, 2, 3), RX_PI)
        self.assertEqual(flange_target_from_tcp(tcp, Pose((0, 0, 0), (0, 0, 0, 1))).position, (1.0, 2.0, 3.0))

    def test_trajectory_conversion_uses_injected_ik(self):
        step = MotionStep(MotionKind.MOVE_L, Pose((1, 2, 3), (0, 0, 0, 1)))
        result = convert_motion_steps_to_ll(
            [step], Pose((0, 0, 0), (0, 0, 0, 1)),
            lambda _pose: [1, 2, 3, 4, 5, 6], (0, 0, 0), (0, 1, 0)
        )
        self.assertEqual(result[0].track_m, 2.0)
        self.assertEqual(result[0].joints_rad, (1, 2, 3, 4, 5, 6))


if __name__ == "__main__":
    unittest.main()
