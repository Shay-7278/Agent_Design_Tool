"""Measured arrival must tolerate delayed convergence without accepting stale TF."""
import math
import tempfile
import unittest
from contextlib import ExitStack
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from geometry_msgs.msg import TransformStamped

from abb_irb4600_gazebo.execute_one_brick import OneBrickExecutor
from abb_irb4600_gazebo.motion import MotionKind, Pose
from abb_irb4600_gazebo.target_motion import (
    quaternion_multiply, rpy_to_quaternion, tcp_target_from_brick,
)


class TcpArrivalTests(unittest.TestCase):
    def exercise(self, mode, *, timeout=10.0):
        node = OneBrickExecutor.__new__(OneBrickExecutor)
        node.sim_time = 100.0
        clock = {'wall': 0.0, 'ticks': 0}
        target = Pose((.4, 2.0, .5), rpy_to_quaternion(0, 0, .3))
        expected = tcp_target_from_brick(target)

        def tick(*args, **kwargs):
            clock['ticks'] += 1
            if clock['ticks'] > 2000:
                raise AssertionError('arrival wait did not remain bounded')
            clock['wall'] += 1.0
            if mode == 'reset':
                node.sim_time = 99.0
            elif mode != 'stalled':
                node.sim_time += .01 if mode == 'slow' else .05

        def lookup(*args, **kwargs):
            elapsed = node.sim_time - 100.0
            position_error = .01
            orientation_error = 0.0
            if mode == 'late' and elapsed >= 1.0:
                position_error = .001
            elif mode == 'orientation':
                position_error = .001
                orientation_error = math.radians(5) if elapsed < .75 else 0.0
            elif mode == 'slow' and elapsed >= .5:
                position_error = .001
            elif mode == 'stale' and elapsed >= .5:
                position_error = .001
            elif mode == 'outside':
                position_error = .004
            tf = TransformStamped()
            stamp = 100.0 if mode == 'stale' else node.sim_time
            ns = round(stamp * 1e9)
            tf.header.stamp.sec, tf.header.stamp.nanosec = divmod(ns, 1000000000)
            t, q = tf.transform.translation, tf.transform.rotation
            t.x, t.y, t.z = (expected.position[0] + position_error,
                             expected.position[1], expected.position[2])
            q.x, q.y, q.z, q.w = quaternion_multiply(
                expected.orientation_xyzw, rpy_to_quaternion(0, 0, orientation_error))
            return tf

        node.tf_buffer = SimpleNamespace(lookup_transform=lookup)
        with tempfile.TemporaryDirectory() as directory, ExitStack() as stack:
            node.observations = Path(directory) / 'assembly_observations.jsonl'
            stack.enter_context(patch('abb_irb4600_gazebo.execute_one_brick.time.monotonic',
                                      side_effect=lambda: clock['wall']))
            stack.enter_context(patch('abb_irb4600_gazebo.execute_one_brick.rclpy.spin_once',
                                      side_effect=tick))
            stack.enter_context(patch('abb_irb4600_gazebo.execute_one_brick.rclpy.ok', return_value=True))
            stack.enter_context(patch.object(OneBrickExecutor, 'get_logger', return_value=Mock()))
            error = None
            try:
                node._wait_for_tcp(target, timeout_sim=timeout)
            except RuntimeError as exc:
                error = exc
            traces = list(Path(directory).glob('*tcp*'))
            self.assertTrue(traces, 'arrival result and error trace were not written')
            self.assertTrue(any(p.stat().st_size for p in traces))
        return node.sim_time - 100.0, clock, error

    def test_late_arrival_passes_after_the_previous_point_two_second_cutoff(self):
        elapsed, _, error = self.exercise('late')
        self.assertIsNone(error)
        self.assertGreaterEqual(elapsed, 1.0)
        self.assertLess(elapsed, 2.0)

    def test_position_inside_limit_does_not_bypass_orientation_limit(self):
        elapsed, _, error = self.exercise('orientation')
        self.assertIsNone(error)
        self.assertGreaterEqual(elapsed, .75)

    def test_four_mm_position_error_times_out_without_relaxing_three_mm_limit(self):
        elapsed, _, error = self.exercise('outside')
        self.assertIsInstance(error, RuntimeError)
        self.assertGreaterEqual(elapsed, 10.0)
        self.assertLess(elapsed, 10.2)

    def test_stale_transform_cannot_confirm_arrival(self):
        elapsed, _, error = self.exercise('stale')
        self.assertIsInstance(error, RuntimeError)
        self.assertGreaterEqual(elapsed, 10.0)

    def test_slow_advancing_clock_can_take_more_than_thirty_real_seconds(self):
        _, clock, error = self.exercise('slow')
        self.assertIsNone(error)
        self.assertGreater(clock['wall'], 30.0)

    def test_stopped_clock_has_a_real_time_bound(self):
        elapsed, clock, error = self.exercise('stalled')
        self.assertIsInstance(error, RuntimeError)
        self.assertIn('clock', str(error).lower())
        self.assertEqual(elapsed, 0.0)
        self.assertLessEqual(clock['wall'], 32.0)

    def test_clock_reset_is_rejected(self):
        _, _, error = self.exercise('reset')
        self.assertIsInstance(error, RuntimeError)
        self.assertIn('reset', str(error).lower())

    def test_failed_arrival_does_not_advance_planned_execution_state(self):
        node = OneBrickExecutor.__new__(OneBrickExecutor)
        node.seed = (0.1,) * 6
        node.last_rail = .5
        node.last_target = Pose((0, 0, 1), (0, 0, 0, 1))
        before = node.seed, node.last_rail, node.last_target
        target = Pose((1, 0, 1), (0, 0, 0, 1))
        node.arm = Mock()
        point = SimpleNamespace(positions=[1.0] * 7)
        with patch.object(OneBrickExecutor, 'get_logger', return_value=Mock()), \
             patch.object(OneBrickExecutor, '_plan_motion',
                          return_value=([point], 1.0, (0.2,) * 6)), \
             patch.object(OneBrickExecutor, 'send'), \
             patch.object(OneBrickExecutor, '_wait_for_tcp', side_effect=RuntimeError('not arrived')):
            with self.assertRaisesRegex(RuntimeError, 'not arrived'):
                node._move_target(target, Pose((0, 0, .24313), (0, 0, 0, 1)), MotionKind.MOVE_L)
        self.assertEqual((node.seed, node.last_rail, node.last_target), before)


if __name__ == '__main__':
    unittest.main()
