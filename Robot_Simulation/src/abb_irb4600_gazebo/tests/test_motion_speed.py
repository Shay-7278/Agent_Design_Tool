"""Speed scaling changes timing while preserving paths and actuator bounds."""
import argparse
import math
import unittest

from abb_irb4600_gazebo.execute_one_brick import OneBrickExecutor, _positive_speed_scale
from abb_irb4600_gazebo.motion import MotionKind, Pose


class MotionSpeedTests(unittest.TestCase):
    def executor(self, scale, rail_limit=100.0):
        node = OneBrickExecutor.__new__(OneBrickExecutor)
        node.speed_scale = scale
        node.profile = {'move_j_speed_m_s': .5, 'move_l_speed_m_s': .05,
                        'orientation_speed_rad_s': 1.0}
        node.last_rail = 0.0
        node.velocity_limits = [rail_limit] + [100.0] * 6
        node._solve = lambda target, seed: (0.0, (target.position[0],) + (0.0,) * 5)
        return node

    @staticmethod
    def seconds(point):
        return point.time_from_start.sec + point.time_from_start.nanosec * 1e-9

    def test_double_speed_halves_linear_timing_without_changing_waypoints(self):
        start = Pose((0, 0, 1), (0, 0, 0, 1))
        target = Pose((.2, 0, 1), (0, 0, 0, 1))
        normal = self.executor(1)._plan_motion(start, target, MotionKind.MOVE_L, (0,) * 6)
        fast = self.executor(2)._plan_motion(start, target, MotionKind.MOVE_L, (0,) * 6)
        self.assertAlmostEqual(normal[1], 7.5)
        self.assertAlmostEqual(fast[1], 3.75)
        self.assertEqual(normal[2], fast[2])
        self.assertEqual(len(normal[0]), len(fast[0]))
        for before, after in zip(normal[0], fast[0]):
            self.assertEqual(before.positions, after.positions)
            self.assertAlmostEqual(self.seconds(before), 2 * self.seconds(after), places=8)

    def test_double_speed_scales_rotation_timing(self):
        start = Pose((0, 0, 1), (0, 0, 0, 1))
        angle = 1.2
        target = Pose(start.position, (0, 0, math.sin(angle / 2), math.cos(angle / 2)))
        normal = self.executor(1)._plan_motion(start, target, MotionKind.MOVE_J, (0,) * 6)
        fast = self.executor(2)._plan_motion(start, target, MotionKind.MOVE_J, (0,) * 6)
        self.assertAlmostEqual(normal[1], 1.875 * angle)
        self.assertAlmostEqual(fast[1], normal[1] / 2)
        self.assertEqual([p.positions for p in normal[0]], [p.positions for p in fast[0]])

    def test_rail_velocity_limit_still_caps_scaled_motion(self):
        start = Pose((0, 0, 1), (0, 0, 0, 1))
        target = Pose((1, 0, 1), (0, 0, 0, 1))
        results = []
        for scale in (1, 2):
            node = self.executor(scale, rail_limit=.25)
            node._solve = lambda target, seed: (target.position[0], (0.0,) * 6)
            points, duration, _ = node._plan_motion(start, target, MotionKind.MOVE_J, (0,) * 6)
            results.append(duration)
            previous_values, previous_time = [0.0] * 7, 0.0
            for point in points:
                stamp = self.seconds(point)
                self.assertGreater(stamp, previous_time)
                for old, value, limit in zip(previous_values, point.positions, node.velocity_limits):
                    self.assertLessEqual(abs(value - old) / (stamp - previous_time), limit + 1e-8)
                previous_values, previous_time = point.positions, stamp
        self.assertGreater(results[1], 7.4)
        self.assertAlmostEqual(results[0], results[1])

    def test_short_move_keeps_minimum_segment_duration(self):
        start = Pose((0, 0, 1), (0, 0, 0, 1))
        target = Pose((.005, 0, 1), (0, 0, 0, 1))
        for scale in (1, 2):
            _, duration, _ = self.executor(scale)._plan_motion(start, target, MotionKind.MOVE_L, (0,) * 6)
            self.assertAlmostEqual(duration, .5)

    def test_speed_scale_accepts_only_positive_finite_values(self):
        for value in ('1', '2', '.5'):
            self.assertEqual(_positive_speed_scale(value), float(value))
        for value in ('0', '-1', 'nan', 'inf', '-inf', 'not-a-number'):
            with self.subTest(value=value), self.assertRaises(argparse.ArgumentTypeError):
                _positive_speed_scale(value)


if __name__ == '__main__':
    unittest.main()
