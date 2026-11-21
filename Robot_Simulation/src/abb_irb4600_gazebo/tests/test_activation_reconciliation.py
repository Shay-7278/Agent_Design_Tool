"""A lost create reply must be reconciled without recreating a supply brick."""
import math
import subprocess
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from abb_irb4600_gazebo.gz_brick_world import (
    GazeboBrickWorld, GazeboCommandError, GazeboRequestTimeout, STATIC_COMPONENT_TYPE,
    COLLISION_COMPONENT_TYPE, PARENT_ENTITY_COMPONENT_TYPE, GEOMETRY_COMPONENT_TYPE,
)
from abb_irb4600_gazebo.motion import Pose
from gz.msgs10.geometry_pb2 import Geometry
from gz.msgs10.scene_pb2 import Scene
from gz.msgs10.serialized_map_pb2 import SerializedStepMap
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.boolean_pb2 import Boolean
from abb_irb4600_gazebo.persistent_service import ServiceTimeout, ServiceRejected


NAME = 'pickup_brick_023'
POSE = Pose((.4, 2., .0725), (0., 0., 0., 1.))


def model(entity_id=20, half=False):
    scene = Scene()
    result = scene.model.add(name=NAME, id=entity_id)
    link = result.link.add(name='brick_link', id=entity_id + 1)
    collision = link.collision.add(name='collision')
    collision.geometry.type = Geometry.BOX
    size = collision.geometry.box.size
    size.x, size.y, size.z = (.095 if half else .190), .090, .057
    return result


def snapshot(entity_id=20, flag=False, source_model=None):
    result = SerializedStepMap()
    entity = result.state.entities[entity_id]
    if flag is not None:
        component = entity.components[1]
        component.type = STATIC_COMPONENT_TYPE
        component.component = b'1' if flag else b'0'
    if source_model is not None:
        collision_id = 100
        for link in source_model.link:
            for collision in link.collision:
                entity = result.state.entities[collision_id]
                for key, component_type, value in (
                        (1, COLLISION_COMPONENT_TYPE, b'-'),
                        (2, PARENT_ENTITY_COMPONENT_TYPE, str(link.id).encode()),
                        (3, GEOMETRY_COMPONENT_TYPE, collision.geometry.SerializeToString())):
                    component = entity.components[key]
                    component.type, component.component = component_type, value
                collision_id += 1
    return result


class ActivationReconciliationTests(unittest.TestCase):
    def world(self, half=False):
        result = GazeboBrickWorld.__new__(GazeboBrickWorld)
        result.world_name = 'test_world'
        root = Path(__file__).resolve().parents[1] / 'models'
        result.brick_sdf = str(root / 'brick/model.sdf')
        result.brick_models = {NAME: root / 'brick_half/model.sdf'} if half else {}
        result.static_supply = True
        result.placed = {}
        result._states = {}
        result._condition = threading.Condition()
        result._poses = {NAME: POSE}
        result._pose_sequence = {NAME: 1}
        result._pose_time = {NAME: (1., 0.)}
        result._pose_ids = {NAME: 20}
        result._link_ids = {}
        result.pose = Mock(return_value=POSE)
        return result

    def verify(self, world, current, flag=False):
        state = snapshot(current.id, flag, current)
        # Real Scene messages omit collision geometry; the ECM is authoritative.
        for link in current.link:
            del link.collision[:]
        world._request = Mock(return_value=state)
        return world._verify_dynamic_supply(current, NAME, POSE)

    def test_full_and_half_models_require_explicit_dynamic_ecm_flag(self):
        for half in (False, True):
            for flag in (False, True, None):
                with self.subTest(half=half, static_flag=flag):
                    world = self.world(half)
                    if flag is False:
                        self.verify(world, model(half=half), flag)
                    else:
                        with self.assertRaises(GazeboCommandError):
                            self.verify(world, model(half=half), flag)

    def test_static_flag_readback_preserves_unknown_instead_of_treating_it_as_dynamic(self):
        world = self.world()
        for flag in (False, True, None):
            world._request = Mock(return_value=snapshot(flag=flag))
            self.assertIs(world._static_flag(20), flag)

    def test_incomplete_or_wrong_model_geometry_is_rejected(self):
        for defect in ('missing_link', 'duplicate_link', 'wrong_shape', 'wrong_size',
                       'extra_collision', 'wrong_pose_id'):
            with self.subTest(defect=defect):
                world = self.world(half=True)
                current = model(half=True)
                if defect == 'missing_link':
                    del current.link[:]
                elif defect == 'duplicate_link':
                    current.link.add(name='brick_link')
                elif defect == 'wrong_shape':
                    current.link[0].collision[0].geometry.type = Geometry.SPHERE
                    current.link[0].collision[0].geometry.ClearField('box')
                    current.link[0].collision[0].geometry.sphere.radius = .05
                elif defect == 'wrong_size':
                    current.link[0].collision[0].geometry.box.size.x = .190
                elif defect == 'extra_collision':
                    current.link[0].collision.add(name='unexpected')
                else:
                    world._pose_ids[NAME] = 10
                with self.assertRaises(GazeboCommandError):
                    self.verify(world, current)

    def test_replacement_pose_must_match_before_activation_can_succeed(self):
        for actual in (Pose((POSE.position[0] + .006, *POSE.position[1:]), POSE.orientation_xyzw),
                       Pose(POSE.position, (0., 0., math.sin(math.radians(3)/2),
                                            math.cos(math.radians(3)/2)))):
            with self.subTest(actual=actual):
                world = self.world()
                world.pose.return_value = actual
                with self.assertRaises(GazeboCommandError):
                    self.verify(world, model())

    def activation(self, outcome):
        world = self.world(half=True)
        current = {'model': model(entity_id=10, half=True), 'flag': True, 'created': False}
        mutations = []
        world.models = Mock(side_effect=lambda: ({NAME: current['model'].id}
                                                if current['model'] is not None else {}))

        def request(service, payload, response_type=None):
            if service == 'remove/blocking':
                self.assertEqual(payload.id, 10)
                mutations.append(('remove', NAME))
                current['model'] = None
                return
            if service == 'create':
                mutations.append(('create', payload.name))
                if outcome == 'explicit_rejection':
                    raise GazeboCommandError('create rejected')
                current['created'] = True
                current['model'] = None if outcome == 'missing' else model(
                    entity_id=10 if outcome == 'old_id' else 20, half=outcome != 'wrong_size')
                current['flag'] = True if outcome == 'static' else (None if outcome == 'missing_static' else False)
                world._pose_ids[NAME] = 10 if outcome in ('old_id', 'stale_pose') else 20
                raise GazeboRequestTimeout('Service call timed out')
            if service == 'scene/info':
                result = Scene()
                if current['model'] is not None:
                    result.model.add().CopyFrom(current['model'])
                    for link in result.model[0].link:
                        del link.collision[:]
                return result
            if service == 'state':
                return snapshot(current['model'].id, current['flag'], current['model'])
            raise AssertionError(service)

        world._request = request
        world.pose.side_effect = lambda name: (
            Pose((POSE.position[0] + .02, *POSE.position[1:]), POSE.orientation_xyzw)
            if current['created'] and outcome == 'wrong_pose' else POSE)

        def wait(predicate, description, timeout=15.):
            if not predicate():
                raise GazeboCommandError('new dynamic entity pose not confirmed')

        world._wait = Mock(side_effect=wait)
        ticks = iter(float(value) for value in range(1000))
        with patch('abb_irb4600_gazebo.gz_brick_world.time.monotonic', side_effect=lambda: next(ticks)), \
             patch('abb_irb4600_gazebo.gz_brick_world.time.sleep'):
            error = None
            try:
                world.activate(NAME)
            except GazeboCommandError as exc:
                error = exc
        self.assertEqual(mutations.count(('create', NAME)), 1)
        self.assertEqual(mutations.count(('remove', NAME)), 1)
        return error, world

    def test_lost_create_reply_accepts_only_verified_dynamic_replacement(self):
        error, world = self.activation('success')
        self.assertIsNone(error)
        self.assertEqual(world._pose_ids[NAME], 20)

    def test_ambiguous_create_is_never_retried_or_claimed_when_readback_is_invalid(self):
        for outcome in ('missing', 'old_id', 'wrong_size', 'static', 'missing_static',
                        'stale_pose', 'wrong_pose'):
            with self.subTest(outcome=outcome):
                error, _ = self.activation(outcome)
                self.assertIsInstance(error, GazeboCommandError)

    def test_explicit_create_rejection_is_not_reclassified_as_a_lost_reply(self):
        error, world = self.activation('explicit_rejection')
        self.assertIsInstance(error, GazeboCommandError)
        self.assertNotIsInstance(error, GazeboRequestTimeout)
        self.assertGreaterEqual(world.models.call_count, 2)

    def test_create_client_timeout_does_not_repeat_mutation(self):
        world = self.world()
        world._service_client = Mock()
        world._service_client.request.side_effect = ServiceTimeout('lost reply')
        with self.assertRaises(GazeboRequestTimeout):
            world._request('create', EntityFactory(name=NAME))
        self.assertEqual(world._service_client.request.call_count, 1)

    def test_persistent_client_distinguishes_timeout_from_explicit_rejection(self):
        world = self.world()
        world._service_client = Mock()
        for error, expected in ((ServiceTimeout('lost'), GazeboRequestTimeout),
                                (ServiceRejected('rejected'), GazeboCommandError)):
            world._service_client.request.side_effect = error
            with self.assertRaises(expected) as caught:
                world._request('create', EntityFactory(name=NAME))
            if isinstance(error, ServiceRejected):
                self.assertNotIsInstance(caught.exception, GazeboRequestTimeout)
        self.assertEqual(world._service_client.request.call_count, 2)

    def test_false_boolean_reply_is_an_explicit_failure_not_an_ambiguous_timeout(self):
        world = self.world()
        world._service_client = Mock()
        world._service_client.request.return_value = Boolean(data=False)
        with self.assertRaises(GazeboCommandError) as caught:
            world._request('create', EntityFactory(name=NAME))
        self.assertNotIsInstance(caught.exception, GazeboRequestTimeout)
        self.assertEqual(world._service_client.request.call_count, 1)

    def test_create_once_returns_uncertain_outcome_without_retrying_or_hiding_rejection(self):
        world = self.world()
        payload = EntityFactory(name=NAME)
        timeout = GazeboRequestTimeout('lost reply')
        world._request = Mock(side_effect=timeout)
        self.assertIs(world._create_once(payload), timeout)
        world._request.assert_called_once_with('create', payload)
        world._request = Mock(side_effect=GazeboCommandError('explicit rejection'))
        with self.assertRaises(GazeboCommandError) as caught:
            world._create_once(payload)
        self.assertNotIsInstance(caught.exception, GazeboRequestTimeout)
        world._request.assert_called_once_with('create', payload)


if __name__ == '__main__':
    unittest.main()
