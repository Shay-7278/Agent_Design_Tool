"""Attachment protocol failures: lost notices, stale notices and clock bounds."""
import threading
import unittest
from unittest.mock import Mock, patch

from gz.msgs10.serialized_map_pb2 import SerializedStepMap
from abb_irb4600_gazebo.gz_brick_world import (
    DETACHABLE_JOINT_TYPE, GazeboBrickWorld, GazeboCommandError, detachable_joints,
)


def snapshot(attached, parent=38, child=60):
    msg = SerializedStepMap()
    msg.state.entities[parent].id = parent
    msg.state.entities[child].id = child
    if attached:
        component = msg.state.entities[500].components[1]
        component.type = DETACHABLE_JOINT_TYPE
        component.component = f'{parent} {child} fixed'.encode()
    return msg


def adapter():
    world = GazeboBrickWorld.__new__(GazeboBrickWorld)
    world._condition = threading.Condition()
    world._sim_time = 0.0
    world._clock_progress_wall = 0.0
    world._link_ids = {'brick': (10, 38, 60)}
    world._states = {}
    world._notifications = {}
    world.attachment_checks = {}
    world._publishers = {('brick', action): Mock() for action in ('attach', 'detach')}
    return world


class AttachmentReadbackTests(unittest.TestCase):
    def test_joint_readback_confirms_attachment_without_any_notification(self):
        world = adapter()
        world._request = Mock(return_value=snapshot(True))
        world._wait_for_joint('brick', True)
        self.assertEqual(world._states['brick'], 'attached')
        self.assertEqual(world.attachment_checks['brick']['joint_entity'], 500)
        self.assertIsNone(world.attachment_checks['brick']['notification'])

    def test_stale_attached_notice_cannot_override_actual_detachment(self):
        world = adapter()
        world._notifications['brick'] = 'attached'
        world._request = Mock(return_value=snapshot(False))
        world._wait_for_joint('brick', False)
        self.assertEqual(world._states['brick'], 'detached')
        self.assertIsNone(world.attachment_checks['brick']['joint_entity'])

    def test_wrong_parent_or_missing_link_is_not_a_confirmed_grasp(self):
        world = adapter()
        world._request = Mock(return_value=snapshot(True, parent=99))
        world._request.return_value.state.entities[38].id = 38
        with self.assertRaisesRegex(GazeboCommandError, 'unexpected physical joint'):
            world._joint_readback('brick')
        world._request.return_value = snapshot(False)
        del world._request.return_value.state.entities[60]
        with self.assertRaisesRegex(GazeboCommandError, 'link disappeared'):
            world._joint_readback('brick')

    def test_slow_progressing_attachment_can_take_more_than_15_real_seconds(self):
        world = adapter()
        clock = {'wall': 0.0, 'reads': 0}
        def readback(name):
            clock['wall'] += 10
            clock['reads'] += 1
            world._sim_time += .5
            world._clock_progress_wall = clock['wall']
            return (500 if clock['reads'] == 4 else None), world._sim_time
        world._joint_readback = readback
        with patch('abb_irb4600_gazebo.gz_brick_world.time.monotonic', side_effect=lambda: clock['wall']):
            world._wait_for_joint('brick', True)
        self.assertEqual(clock['wall'], 40)
        self.assertEqual(world._publishers['brick', 'attach'].publish.call_count, 3)

    def test_advancing_clock_has_a_simulation_time_budget(self):
        world = adapter()
        def readback(name):
            world._sim_time += 6
            return None, world._sim_time
        world._joint_readback = readback
        with self.assertRaisesRegex(GazeboCommandError, '10.0 simulation seconds'):
            world._wait_for_joint('brick', True)
        self.assertNotIn('brick', world._states)

    def test_stalled_clock_is_bounded_even_when_readback_keeps_returning(self):
        world = adapter()
        clock = {'wall': 0.0}
        def readback(name):
            clock['wall'] += 16
            return None, 0.0
        world._joint_readback = readback
        with patch('abb_irb4600_gazebo.gz_brick_world.time.monotonic', side_effect=lambda: clock['wall']):
            with self.assertRaisesRegex(GazeboCommandError, 'stopped for 30.0 real seconds'):
                world._wait_for_joint('brick', True)

    def test_component_parser_rejects_malformed_joint_data(self):
        msg = snapshot(True)
        msg.state.entities[500].components[1].component = b'not a joint'
        with self.assertRaises(GazeboCommandError):
            detachable_joints(msg)
