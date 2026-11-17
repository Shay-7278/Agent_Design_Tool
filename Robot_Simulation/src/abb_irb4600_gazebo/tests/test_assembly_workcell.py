"""Verify that the smaller workcell preserves the executing robot and supports."""

import hashlib
import json
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory


def signature(element):
    return (element.tag, sorted(element.attrib.items()), (element.text or '').strip(),
            [signature(child) for child in element])


class AssemblyWorkcellTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.gazebo = Path(get_package_share_directory('abb_irb4600_gazebo'))
        cls.description = Path(get_package_share_directory('abb_irb4600_workcell_description'))
        cls.full = ET.fromstring(xacro.process_file(str(
            cls.gazebo / 'urdf/abb_irb4600_workcell.gazebo.xacro')).toxml())
        cls.small = ET.fromstring(xacro.process_file(str(
            cls.gazebo / 'urdf/abb_irb4600_assembly.gazebo.xacro')).toxml())

    def test_only_executing_robot_and_supports_remain(self):
        names = {link.get('name') for link in self.small.findall('link')}
        self.assertTrue({'world', 'workcell_base', 'MT_base', 'platform', 'LL_schunk_tcp'} <= names)
        self.assertTrue(all(name.startswith('LL_') or name in {
            'world', 'workcell_base', 'MT_base', 'platform'} for name in names))
        moving = {joint.get('name') for joint in self.small.findall('joint') if joint.get('type') != 'fixed'}
        expected = {'joint_LL_ext_1', *[f'joint_LL_{i}' for i in range(1, 7)],
                    'LL_schunk_left_finger_joint', 'LL_schunk_right_finger_joint'}
        self.assertEqual(moving, expected)
        self.assertEqual({joint.get('name') for joint in self.small.findall('ros2_control/joint')}, expected)

    def test_robot_track_gripper_meshes_dynamics_and_transforms_are_unchanged(self):
        for tag in ('link', 'joint'):
            for original in self.full.findall(tag):
                name = original.get('name')
                if name.startswith(('LL_', 'joint_LL_')):
                    retained = self.small.find(f"./{tag}[@name='{name}']")
                    self.assertIsNotNone(retained, name)
                    self.assertEqual(signature(original), signature(retained), name)

    def test_support_collisions_and_calibration_are_unchanged(self):
        for query in ["./link[@name='MT_base']/collision", "./joint[@name='MT_fixed_joint']",
                      "./link[@name='platform']", "./joint[@name='platform_fixed_joint']",
                      "./joint[@name='workcell_base_fixed_joint']"]:
            self.assertEqual(signature(self.full.find(query)), signature(self.small.find(query)), query)
        visual = self.small.find("./link[@name='MT_base']/visual/geometry/mesh")
        self.assertTrue(visual.get('filename').endswith('/meshes/optimized/fixtures/MT_base.stl'))

    def test_left_controller_settings_are_identical(self):
        full = yaml.safe_load((self.gazebo / 'config/ros2_controllers.yaml').read_text())
        small = yaml.safe_load((self.gazebo / 'config/assembly_controllers.yaml').read_text())
        for name in ('LL_manipulator_controller', 'LL_gripper_controller'):
            self.assertEqual(full[name], small[name])
        self.assertNotIn('RR_manipulator_controller', small)
        self.assertNotIn('RR_manipulator_controller', small['controller_manager']['ros__parameters'])
        parameters = self.small.find('gazebo/plugin/parameters').text
        self.assertTrue(parameters.endswith('/config/assembly_controllers.yaml'))

    def test_table_visual_is_reduced_without_moving_its_bounds(self):
        meshes = self.description / 'meshes'
        source = meshes / 'visual/fixtures/MT_base.stl'
        output = meshes / 'optimized/fixtures/MT_base.stl'
        report = json.loads(output.with_suffix('.validation.json').read_text())
        self.assertEqual(hashlib.sha256(source.read_bytes()).hexdigest(), report['source_sha256'])
        dtype = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
        old = np.frombuffer(source.read_bytes(), dtype=dtype, offset=84)['vertices'].reshape(-1, 3)
        new = np.frombuffer(output.read_bytes(), dtype=dtype, offset=84)['vertices'].reshape(-1, 3)
        self.assertLess(len(new), len(old) / 10)
        np.testing.assert_allclose([new.min(axis=0), new.max(axis=0)],
                                   [old.min(axis=0), old.max(axis=0)], atol=0.001)


if __name__ == '__main__':
    unittest.main()
