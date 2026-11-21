"""Physical postconditions resolve lost replies without replaying mutations."""
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from abb_irb4600_gazebo.gz_brick_world import GazeboBrickWorld, GazeboCommandError, GazeboRequestTimeout
from abb_irb4600_gazebo.motion import Pose
from abb_irb4600_gazebo.persistent_service import PersistentServiceClient, ServiceClientError, ServiceTimeout


class MutationOutcomeTests(unittest.TestCase):
    def world(self):
        w = GazeboBrickWorld.__new__(GazeboBrickWorld)
        w._sim_time = 10.0
        w._clock_progress_wall = time.monotonic()
        w._condition = threading.Condition()
        w._plugins = set()
        w._publishers = {}
        w._notifications = {}
        w._states = {}
        w.attachment_checks = {}
        w._link_ids = {'brick': (10, 38, 41)}
        w._poses = {}
        w._pose_ids = {}
        w._pose_sequence = {}
        w.parent_model = 'robot'
        w.parent_link = 'flange'
        w.node = Mock()
        return w

    def test_attachment_success_with_lost_reply_confirms_joint_and_never_reloads_plugin(self):
        w = self.world()
        w._request = Mock(side_effect=GazeboRequestTimeout('reply lost after loading'))
        w._joint_readback = Mock(return_value=(492, 10.0))
        w.attach('brick')
        self.assertEqual(w.attachment_checks['brick']['joint_entity'], 492)
        self.assertEqual(w._states['brick'], 'attached')
        self.assertIn('brick', w._plugins)
        w.attach('brick')
        self.assertEqual(w._request.call_count, 1)
        self.assertEqual(w._request.call_args.args[0], 'entity/system/add')

    def test_explicit_plugin_rejection_cannot_claim_attachment(self):
        w = self.world()
        w._request = Mock(side_effect=GazeboCommandError('rejected'))
        w._joint_readback = Mock()
        with self.assertRaisesRegex(GazeboCommandError, 'rejected'):
            w.attach('brick')
        w._joint_readback.assert_not_called()
        self.assertNotIn('brick', w._plugins)

    def test_lost_remove_reply_checks_original_id_without_deleting_reused_name(self):
        w = self.world()
        entities = {'brick': 32}
        w.models = lambda: dict(entities)
        requests = []
        def remove(service, request):
            requests.append((service, request.id))
            entities['brick'] = 33  # original ID gone; a newer body owns the name
            raise GazeboRequestTimeout('removed but reply lost')
        w._request = remove
        w._remove_confirmed('brick')
        self.assertEqual(requests, [('remove/blocking', 32)])
        self.assertEqual(entities, {'brick': 33})

    def test_lost_pose_update_reply_requires_new_feedback_from_same_id(self):
        w = self.world()
        target = Pose((1, 2, .0725), (0, 0, 0, 1))
        w.models = Mock(return_value={'brick': 32})
        w._pose_ids['brick'] = 32
        w._pose_sequence['brick'] = 1
        w.pose = Mock(return_value=target)
        calls = []
        def update(service, request):
            calls.append((service, request.id))
            w._pose_sequence['brick'] = 2
            raise GazeboRequestTimeout('moved but reply lost')
        w._request = update
        w.set_pose('brick', target)
        self.assertEqual(calls, [('set_pose', 32)])

    def test_transient_read_failure_does_not_abort_verification_before_deadline(self):
        w = self.world()
        check = Mock(side_effect=[GazeboRequestTimeout('read lost'), True])
        self.assertTrue(w._confirm(check, 'state', timeout=1))
        self.assertEqual(check.call_count, 2)

    def test_world_reset_rejects_pending_operation(self):
        w = self.world()
        w._operation_start_clock = 11.0
        with self.assertRaisesRegex(GazeboCommandError, 'reset'):
            w._check_operation()

    def test_partial_client_frame_poisoning_prevents_late_reply_reuse(self):
        client = PersistentServiceClient.__new__(PersistentServiceClient)
        client._lock = threading.Lock()
        client._broken = False
        client._process = Mock()
        client._process.poll.return_value = None
        client._read_exact = Mock(side_effect=ServiceTimeout('partial frame'))
        from gz.msgs10.empty_pb2 import Empty
        from gz.msgs10.boolean_pb2 import Boolean
        with self.assertRaises(ServiceTimeout):
            client.request('/test', Empty(), Boolean, timeout_ms=10)
        with self.assertRaises(ServiceClientError):
            client.request('/test', Empty(), Boolean, timeout_ms=10)
        self.assertEqual(client._process.stdin.write.call_count, 1)
        client._process.terminate.assert_called_once()
