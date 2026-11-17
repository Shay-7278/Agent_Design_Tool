"""Exercise trajectory waiting with controlled ROS futures and both clocks."""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from control_msgs.action import FollowJointTrajectory
from abb_irb4600_gazebo.execute_one_brick import OneBrickExecutor, TrajectoryWatchdog


class TrajectoryWaitTests(unittest.TestCase):
    def exercise(self, mode):
        node = OneBrickExecutor.__new__(OneBrickExecutor)
        node.sim_time = 0.0
        clock = {'wall': 0.0, 'done': False, 'callback': None}
        result = Mock()
        result.done.side_effect = lambda: clock['done']
        result.result.return_value = SimpleNamespace(result=SimpleNamespace(error_code=FollowJointTrajectory.Result.SUCCESSFUL))
        handle = Mock(accepted=True)
        handle.get_result_async.return_value = result
        cancellation = Mock()
        cancellation.done.return_value = True
        cancellation.result.return_value = SimpleNamespace(goals_canceling=[1])

        def cancel():
            clock['done'] = True
            return cancellation

        handle.cancel_goal_async.side_effect = cancel
        accepted = Mock()
        accepted.done.return_value = True
        accepted.result.return_value = handle
        client = Mock()
        client.wait_for_server.return_value = True

        def send_goal(goal, feedback_callback):
            clock['callback'] = feedback_callback
            return accepted

        client.send_goal_async.side_effect = send_goal

        def tick(*args, **kwargs):
            clock['wall'] += 1
            if mode != 'stalled':
                node.sim_time += .05 if mode == 'slow' else 1.0
            if mode != 'lost_feedback':
                clock['callback'](None)
            if mode == 'slow' and node.sim_time >= 15:
                clock['done'] = True

        with patch('abb_irb4600_gazebo.execute_one_brick.time.monotonic', side_effect=lambda: clock['wall']), \
             patch('abb_irb4600_gazebo.execute_one_brick.rclpy.spin_once', side_effect=tick), \
             patch('abb_irb4600_gazebo.execute_one_brick.rclpy.spin_until_future_complete'), \
             patch('abb_irb4600_gazebo.execute_one_brick.rclpy.ok', return_value=True), \
             patch.object(OneBrickExecutor, 'get_logger', return_value=Mock()):
            if mode == 'slow':
                node.send(client, ['joint'], [1.0], seconds=15)
                self.assertGreater(clock['wall'], 90)
                handle.cancel_goal_async.assert_not_called()
            else:
                reason = 'clock stopped' if mode == 'stalled' else 'feedback stopped'
                with self.assertRaisesRegex(RuntimeError, reason + '.*cancellation confirmed'):
                    node.send(client, ['joint'], [1.0], seconds=15)
                handle.cancel_goal_async.assert_called_once()

    def test_slow_progressing_goal_can_finish_after_90_real_seconds(self):
        self.exercise('slow')

    def test_stopped_clock_cancels_and_waits_for_acknowledgment(self):
        self.exercise('stalled')

    def test_lost_feedback_with_advancing_clock_cancels(self):
        self.exercise('lost_feedback')

    def test_missing_result_budget_is_measured_in_simulation_seconds(self):
        watchdog = TrajectoryWatchdog(0, 0, 15)
        watchdog.check(24, 240)
        with self.assertRaisesRegex(RuntimeError, 'no result'):
            watchdog.check(26, 260)

    def test_clock_reset_is_detected(self):
        watchdog = TrajectoryWatchdog(20, 0, 15)
        with self.assertRaisesRegex(RuntimeError, 'clock reset'):
            watchdog.check(0, 1)


if __name__ == '__main__':
    unittest.main()
