"""A transient verification read must not replay entity mutations."""
import subprocess
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch
from abb_irb4600_gazebo.persistent_service import ServiceTimeout

from abb_irb4600_gazebo.gz_brick_world import GazeboBrickWorld, GazeboCommandError
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.scene_pb2 import Scene


class GazeboReadRetryTests(unittest.TestCase):
    def world(self):
        world = GazeboBrickWorld.__new__(GazeboBrickWorld)
        world.world_name = 'test_world'
        world._service_client = Mock()
        return world

    def test_scene_verification_retries_a_lost_reply(self):
        world = self.world()
        world._service_client.request.side_effect = [ServiceTimeout('lost'), Scene(name='verified')]
        scene = world._request('scene/info', Empty(), Scene)
        self.assertEqual(scene.name, 'verified')
        self.assertEqual(world._service_client.request.call_count, 2)

    def test_a_process_timeout_also_has_a_bounded_read_retry(self):
        world = self.world()
        world._service_client.request.side_effect = ServiceTimeout('lost')
        with self.assertRaises(GazeboCommandError):
            world._request('scene/info', Empty(), Scene)
        self.assertEqual(world._service_client.request.call_count, 2)

    def test_create_is_never_reissued_after_an_uncertain_reply(self):
        world = self.world()
        world._service_client.request.side_effect = ServiceTimeout('lost')
        with self.assertRaises(GazeboCommandError):
            world._request('create', EntityFactory(name='one_brick'))
        self.assertEqual(world._service_client.request.call_count, 1)
