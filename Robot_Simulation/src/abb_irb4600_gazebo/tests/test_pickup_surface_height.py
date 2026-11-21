"""Fixed supply and pickup targets share the collision surface's resting height."""
import json
import math
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from ament_index_python.packages import get_package_share_directory
from abb_irb4600_gazebo.brick_models import pickup_pose_on_platform
from abb_irb4600_gazebo.brick_lifecycle import load_pickup_layout
from abb_irb4600_gazebo.gz_brick_world import GazeboBrickWorld  # Register transport before protobuf types.
from abb_irb4600_gazebo.motion import Pose
from abb_irb4600_gazebo import startup_brick_spawner
from google.protobuf import text_format
from gz.msgs10.entity_factory_pb2 import EntityFactory


class PickupSurfaceHeightTests(unittest.TestCase):
    def models(self):
        root = Path(__file__).resolve().parents[1] / 'models'
        return (root / 'brick/model.sdf', root / 'brick_half/model.sdf')

    def test_all_fifty_exported_slots_rest_on_platform_for_both_sizes(self):
        description = Path(get_package_share_directory('abb_irb4600_workcell_description'))
        source = description / 'config/fixed_pickup_layout.json'
        before = source.read_bytes()
        raw = load_pickup_layout(source)
        self.assertEqual(len(raw), 50)
        self.assertTrue(all(abs(p.position[2] - .0865) < 1e-12 for p in raw))
        for model in self.models():
            model_before = model.read_bytes()
            for index, pose in enumerate(raw):
                with self.subTest(model=model.parent.name, slot=index + 1):
                    adjusted = pickup_pose_on_platform(pose, model, platform_top=.044)
                    self.assertEqual(adjusted.position[:2], pose.position[:2])
                    self.assertEqual(adjusted.orientation_xyzw, pose.orientation_xyzw)
                    self.assertAlmostEqual(adjusted.position[2], .0725, places=12)
                    self.assertAlmostEqual(adjusted.position[2] - .057 / 2, .044, places=12)
            self.assertEqual(model.read_bytes(), model_before)
        self.assertEqual(source.read_bytes(), before)

    def test_default_support_height_uses_calibrated_platform_configuration(self):
        for model in self.models():
            pose = Pose((.4475558333333332, 2.5116577777777773, .0865), (0, 0, 0, 1))
            adjusted = pickup_pose_on_platform(pose, model)
            self.assertAlmostEqual(adjusted.position[2], .0725, places=12)
            self.assertAlmostEqual(pose.position[2] - adjusted.position[2], .014, places=12)

    def test_height_is_derived_from_collision_geometry_and_preserves_heading(self):
        with tempfile.TemporaryDirectory() as directory:
            model = Path(directory) / 'model.sdf'
            model.write_text(
                '<sdf version="1.9"><model name="custom"><link name="brick_link">'
                '<collision><geometry><box><size>.2 .1 .08</size></box></geometry></collision>'
                '<visual><geometry><box><size>.2 .1 .01</size></box></geometry></visual>'
                '</link></model></sdf>', encoding='utf-8')
            heading = (0, 0, math.sin(.35), math.cos(.35))
            pose = Pose((1.2, -2.3, 99), heading)
            adjusted = pickup_pose_on_platform(pose, model, platform_top=.1)
        self.assertEqual(adjusted.position[:2], pose.position[:2])
        self.assertEqual(adjusted.orientation_xyzw, pose.orientation_xyzw)
        self.assertAlmostEqual(adjusted.position[2], .14, places=12)

    def test_startup_create_request_uses_the_same_resting_height_for_full_and_half(self):
        for model in self.models():
            with self.subTest(model=model.parent.name):
                raw = Pose((1.2, 2.3, .0865), (0, 0, 0, 1))
                before = model.read_bytes()
                adapter = Mock(brick_models={})
                startup_brick_spawner._spawn(adapter, 'pickup_brick_023', model, raw)
                name, sent = adapter.spawn.call_args.args
                self.assertEqual(name, 'pickup_brick_023')
                self.assertEqual(sent.position[:2], raw.position[:2])
                self.assertAlmostEqual(sent.position[2], .0725, places=12)
                self.assertEqual(adapter.brick_models[name], model)
                self.assertEqual(model.read_bytes(), before)


if __name__ == '__main__':
    unittest.main()
