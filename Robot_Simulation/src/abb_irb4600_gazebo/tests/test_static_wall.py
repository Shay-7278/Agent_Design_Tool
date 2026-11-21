"""Fixed assembly geometry preserves size and never claims an unverified swap."""
import math
import threading
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

from abb_irb4600_gazebo.gz_brick_world import GazeboBrickWorld, GazeboCommandError, STATIC_COMPONENT_TYPE
from abb_irb4600_gazebo.motion import Pose


class StaticWallTests(unittest.TestCase):
    def test_native_subscribers_are_closed_before_interpreter_exit(self):
        world = GazeboBrickWorld.__new__(GazeboBrickWorld)
        transport = Mock()
        transport.subscribed_topics.return_value = ['/world/clock', '/world/pose/info', '/assembly/brick/state']
        world.node = transport
        world._publishers = {'attachment': Mock()}
        world.close()
        self.assertEqual(transport.unsubscribe.call_count, 3)
        self.assertIsNone(world.node)
        self.assertFalse(world._publishers)
        world.close()
        self.assertEqual(transport.unsubscribe.call_count, 3)

    def world(self):
        world = GazeboBrickWorld.__new__(GazeboBrickWorld)
        root = Path(__file__).resolve().parents[1] / 'models'
        world.brick_sdf = str(root / 'brick/model.sdf')
        world.brick_models = {'pickup_brick_008': root / 'brick_half/model.sdf'}
        world.placed = {}
        world.static_model_name = None
        world._static_generation = 0
        world._states = {}
        world._poses = {}
        world._pose_sequence = {}
        world._pose_time = {}
        world._pose_ids = {}
        world._link_ids = {}
        world._sim_time = 12.0
        world._condition = threading.Condition()
        models = {'abb_irb4600_workcell': [], 'pickup_brick_000': [], 'pickup_brick_008': []}
        ids = {name: i+1 for i, name in enumerate(models)}
        calls = []
        created = []

        def request(service, payload, response_type=None):
            if service == 'scene/info':
                calls.append(('read',))
                return SimpleNamespace(model=[
                    SimpleNamespace(name=name, id=ids[name], is_static=False,
                                    link=[SimpleNamespace(name=n) for n in names])
                    for i, (name, names) in enumerate(models.items())])
            if service == 'state':
                return SimpleNamespace(state=SimpleNamespace(entities={
                    ids[name]: SimpleNamespace(components={1: SimpleNamespace(
                        type=STATIC_COMPONENT_TYPE,
                        component=b'1' if name.startswith('placed_structure_') else b'0')})
                    for i, name in enumerate(models)}))
            if service == 'remove/blocking':
                name = next(name for name in models if ids[name] == payload.id)
                calls.append(('remove', name))
                del models[name]
                return
            if service == 'create':
                calls.append(('create', payload.name))
                self.assertEqual(payload.sdf, '')
                tree = ET.parse(payload.sdf_filename).getroot()
                model = tree.find('model')
                created.append(tree)
                ids[payload.name] = max(ids.values()) + 1
                models[payload.name] = [link.get('name') for link in model.findall('link')]
                return
            raise AssertionError(service)

        world._request = request
        return world, models, calls, created

    def test_prefix_becomes_one_static_model_with_full_and_half_dimensions(self):
        world, models, calls, created = self.world()
        full = Pose((1, 2, .4), (0, 0, 0, 1))
        half = Pose((-.2, 3, .5), (0, 0, math.sin(.3), math.cos(.3)))
        world.prefill_static([('pickup_brick_000', full), ('pickup_brick_008', half)])
        self.assertEqual(set(models), {'abb_irb4600_workcell', 'placed_structure_0001'})
        model = created[-1].find('model')
        self.assertEqual(model.findtext('static'), 'true')
        links = {link.get('name'): link for link in model.findall('link')}
        for name, length in [('pickup_brick_000', .19), ('pickup_brick_008', .095)]:
            collision = tuple(map(float, links[name].findtext('collision/geometry/box/size').split()))
            visual = tuple(map(float, links[name].findtext('visual/geometry/box/size').split()))
            self.assertEqual(collision, (length, .09, .057))
            self.assertEqual(collision, visual)
            self.assertEqual(links[name].findtext('collision/surface/friction/ode/mu'), '0.84')
        values = list(map(float, links['pickup_brick_008'].findtext('pose').split()))
        self.assertEqual(values[:3], list(half.position))
        self.assertAlmostEqual(values[5], .6)
        manifest = world.static_wall_manifest()
        self.assertEqual([row['name'] for row in manifest], ['pickup_brick_000', 'pickup_brick_008'])
        self.assertEqual(manifest[1]['dimensions_m'], (.095, .09, .057))
        self.assertEqual(world.observation('pickup_brick_008'), (half, 1, 12.0))

    def test_replacement_retains_old_support_until_new_model_is_verified(self):
        world, models, calls, created = self.world()
        world.prefill_static([('pickup_brick_000', Pose((0, 0, .1), (0, 0, 0, 1)))])
        old = world.static_model_name
        calls.clear()
        world.consolidate('pickup_brick_008', Pose((.2, 0, .1), (0, 0, 0, 1)))
        remove_brick = calls.index(('remove', 'pickup_brick_008'))
        create = calls.index(('create', 'placed_structure_0002'))
        remove_old = calls.index(('remove', old))
        self.assertLess(remove_brick, create)
        self.assertLess(create, remove_old)
        self.assertIn(('read',), calls[create+1:remove_old])
        self.assertNotIn(old, models)
        self.assertEqual(len(world.placed), 2)

    def test_model_visibility_does_not_confirm_an_incomplete_link_list(self):
        world, _, _, _ = self.world()
        original = world._request
        partial_reads = []
        def request(service, payload, response_type=None):
            response = original(service, payload, response_type)
            if service == 'scene/info':
                for model in response.model:
                    if model.name.startswith('placed_structure_') and not partial_reads:
                        partial_reads.append(model.name)
                        model.link = []
            return response
        world._request = request
        world.consolidate('pickup_brick_000', Pose((0, 0, .1), (0, 0, 0, 1)))
        self.assertEqual(partial_reads, ['placed_structure_0001'])
        self.assertIn('pickup_brick_000', world.placed)

    def test_221_member_wall_uses_file_request_instead_of_oversized_inline_argument(self):
        world, _, calls, created = self.world()
        world.prefill_static([(f'pickup_brick_{i:03d}', Pose((i*.2, 0, .1), (0, 0, 0, 1)))
                              for i in range(221)])
        self.assertEqual(len(created[-1].findall('./model/link')), 221)
        self.assertGreater(world._static_sdf_path.stat().st_size, 128*1024)
        self.assertEqual(len(world.static_wall_manifest()), 221)

    def test_failed_create_preserves_old_wall_and_does_not_claim_new_member(self):
        world, models, calls, _ = self.world()
        world.prefill_static([('pickup_brick_000', Pose((0, 0, .1), (0, 0, 0, 1)))])
        old = world.static_model_name
        original = world._request
        def fail(service, payload, response_type=None):
            if service == 'create':
                raise GazeboCommandError('create rejected')
            return original(service, payload, response_type)
        world._request = fail
        with self.assertRaisesRegex(GazeboCommandError, 'create rejected'):
            world.consolidate('pickup_brick_008', Pose((.2, 0, .1), (0, 0, 0, 1)))
        self.assertIn(old, models)
        self.assertEqual(world.static_model_name, old)
        self.assertNotIn('pickup_brick_008', world.placed)

    def test_attached_or_nonbrick_entities_are_rejected_before_mutation(self):
        world, models, calls, _ = self.world()
        world._states['pickup_brick_008'] = 'attached'
        for name in ['pickup_brick_008', 'abb_irb4600_workcell']:
            with self.subTest(name=name), self.assertRaises(GazeboCommandError):
                world.consolidate(name, Pose((0, 0, .1), (0, 0, 0, 1)))
        self.assertEqual(calls, [])

    def test_unset_scene_static_field_uses_confirmed_ecm_static_component(self):
        world, _, _, _ = self.world()
        # The fake Scene reports is_static=False, matching live Gazebo's unset
        # field. Its actual ECM Static flag is true, so consolidation succeeds.
        world.consolidate('pickup_brick_000', Pose((0, 0, .1), (0, 0, 0, 1)))
        self.assertEqual(world.static_model_name, 'placed_structure_0001')

    def test_false_ecm_static_flag_cannot_be_accepted_from_scene_geometry(self):
        world, models, _, _ = self.world()
        original = world._request
        def false_static(service, payload, response_type=None):
            response = original(service, payload, response_type)
            if service == 'state':
                for entity in response.state.entities.values():
                    for component in entity.components.values():
                        component.component = b'0'
            return response
        world._request = false_static
        with self.assertRaisesRegex(GazeboCommandError, 'is not static'):
            world.consolidate('pickup_brick_000', Pose((0, 0, .1), (0, 0, 0, 1)))
        self.assertEqual(world.placed, {})
        self.assertIsNone(world.static_model_name)

    def test_supply_boxes_are_static_only_in_assembly_mode(self):
        world, _, _, _ = self.world()
        world._request = Mock()
        world.models = Mock(return_value={})
        world._confirm = Mock(return_value=108)
        world.pose = Mock()
        pose = Pose((1, 2, .1), (0, 0, 0, 1))
        world.static_supply = False
        world.spawn('pickup_brick_008', pose)
        request = world._request.call_args.args[1]
        self.assertTrue(request.sdf_filename.endswith('brick_half/model.sdf'))
        self.assertEqual(request.sdf, '')
        self.assertAlmostEqual(request.pose.position.z, .1)
        world.static_supply = True
        world.spawn('pickup_brick_008', pose)
        request = world._request.call_args.args[1]
        model = ET.fromstring(request.sdf).find('model')
        self.assertEqual(model.findtext('static'), 'true')
        self.assertEqual(tuple(map(float, model.findtext('link/collision/geometry/box/size').split())),
                         (.095, .09, .057))
        self.assertEqual(request.pose.position.x, 1)
        self.assertAlmostEqual(request.pose.position.z, .0725)

    def test_activating_supply_uses_dynamic_model_and_rejects_stale_old_entity_pose(self):
        world, _, _, _ = self.world()
        name = 'pickup_brick_008'
        pose = Pose((1, 2, .1), (0, 0, 0, 1))
        model = SimpleNamespace(name=name, id=108, link=[SimpleNamespace(name='brick_link')])
        world._request = Mock(return_value=SimpleNamespace(model=[model]))
        world._pose_ids[name] = 8
        world._verify_supply = Mock()
        self.assertFalse(world._supply_readback(name, pose, False, old_id=8))
        world._verify_supply.assert_not_called()
        world._pose_ids[name] = 108
        self.assertEqual(world._supply_readback(name, pose, False, old_id=8), 108)
        world._verify_supply.assert_called_once_with(model, name, pose, False)
