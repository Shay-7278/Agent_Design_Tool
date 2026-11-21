"""Dynamic assessment preserves geometry and cannot gate assembly execution."""
import contextlib
import io
import json
from pathlib import Path
from types import SimpleNamespace
import sys
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

from abb_irb4600_gazebo import assess_stability as assessment


PACKAGE = Path(__file__).resolve().parents[1]
DESCRIPTION = PACKAGE.parent/'abb_irb4600_workcell_description'
if not DESCRIPTION.is_dir():
    from ament_index_python.packages import get_package_share_directory
    DESCRIPTION = Path(get_package_share_directory('abb_irb4600_workcell_description'))


def manifest():
    return {'frame': 'workcell_base', 'mode': 'assembly_visualization', 'bricks': [
        {'name': 'brick_full', 'position': [0., 0., .4], 'orientation_xyzw': [0., 0., 0., 1.],
         'model_sdf': str(PACKAGE/'models/brick/model.sdf'), 'dimensions_m': [.19, .09, .057]},
        {'name': 'brick_half', 'position': [.2, 0., .4], 'orientation_xyzw': [0., 0., 0., 1.],
         'model_sdf': str(PACKAGE/'models/brick_half/model.sdf'), 'dimensions_m': [.095, .09, .057]},
    ]}


class StabilityAssessmentTests(unittest.TestCase):
    def test_assessment_keeps_dynamic_full_half_brick_parameters_and_support_geometry(self):
        root = ET.fromstring(assessment.build_assessment_world(manifest(), PACKAGE, DESCRIPTION, 'isolated_test'))
        world = root.find('world')
        self.assertEqual(world.get('name'), 'isolated_test')
        self.assertEqual(world.findtext('physics/max_step_size'), '0.001')
        self.assertEqual(len(world.findall("model[@name='abb_irb4600_workcell']")), 0)
        for name, length, mass in [('brick_full', .19, .68229), ('brick_half', .095, .341145)]:
            model = world.find(f"model[@name='{name}']")
            self.assertEqual(model.findtext('static'), 'false')
            self.assertAlmostEqual(float(model.findtext('link/inertial/mass')), mass)
            size = list(map(float, model.findtext('link/collision/geometry/box/size').split()))
            self.assertEqual(size, [length, .09, .057])
            self.assertEqual(model.findtext('link/collision/surface/friction/ode/mu'), '0.84')
        table = world.find("model[@name='assessment_MT_support']")
        self.assertEqual(table.findtext('static'), 'true')
        self.assertEqual(table.findtext('pose'), '-1.018799012 4.387074083 0.371454666 0.0 0.0 0.0')
        self.assertTrue(table.findtext('link/collision/geometry/mesh/uri').endswith('/meshes/collision/fixtures/MT_base.stl'))
        platform = world.find("model[@name='assessment_platform']")
        self.assertEqual(list(map(float, platform.findtext('link/collision/geometry/box/size').split())), [5.47, 2.44, .01905])
        self.assertAlmostEqual(float(platform.findtext('pose').split()[2]), .044-.01905/2)

    def test_movement_warning_never_blocks_assembly_or_reports_stable(self):
        data = manifest()
        poses = {b['name']: SimpleNamespace(position=(b['position'][0], 0., .35),
                 orientation_xyzw=(0., 0., 0., 1.)) for b in data['bricks']}
        report = assessment.summarize_assessment(data, poses, 10., 10.)
        self.assertEqual(report['status'], 'WARNING')
        self.assertFalse(report['assembly_blocked'])
        self.assertEqual(len(report['warnings']), 2)
        self.assertAlmostEqual(report['max_displacement_m'], .05)

    def test_transient_displacement_is_retained_even_if_final_pose_returns(self):
        data = manifest()
        poses = {b['name']: SimpleNamespace(position=b['position'], orientation_xyzw=b['orientation_xyzw'])
                 for b in data['bricks']}
        report = assessment.summarize_assessment(data, poses, 10., 10.,
                                                peak_displacements={'brick_half': .08})
        self.assertEqual(report['status'], 'WARNING')
        self.assertEqual(report['max_displacement_m'], 0.)
        self.assertEqual(report['max_observed_displacement_m'], .08)

    def test_missing_or_timed_out_observation_is_incomplete_not_success(self):
        report = assessment.summarize_assessment(manifest(), {}, .2, 10., 'real-time budget reached')
        self.assertEqual(report['status'], 'INCOMPLETE')
        self.assertFalse(report['assembly_blocked'])
        self.assertEqual(len(report['missing_bricks']), 2)

    def test_reported_warning_and_timeout_both_return_zero_from_cli(self):
        for status in ('WARNING', 'INCOMPLETE'):
            with self.subTest(status=status), tempfile.TemporaryDirectory() as directory:
                source = Path(directory)/'manifest.json'
                output = Path(directory)/'report.json'
                source.write_text(json.dumps(manifest()))
                packages = SimpleNamespace(get_package_share_directory=lambda p: str(PACKAGE))
                with patch.dict(sys.modules, {'ament_index_python': SimpleNamespace(packages=packages),
                                             'ament_index_python.packages': packages}), \
                     patch.object(assessment, 'run_assessment', return_value={'status': status, 'assembly_blocked': False}), \
                     contextlib.redirect_stdout(io.StringIO()):
                    result = assessment.main(['--manifest', str(source), '--report', str(output)])
                self.assertEqual(result, 0)
                self.assertEqual(json.loads(output.read_text())['status'], status)

    def test_missing_manifest_is_reported_and_nonblocking(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)/'report.json'
            packages = SimpleNamespace(get_package_share_directory=lambda p: str(PACKAGE))
            with patch.dict(sys.modules, {'ament_index_python': SimpleNamespace(packages=packages),
                                         'ament_index_python.packages': packages}), \
                 contextlib.redirect_stdout(io.StringIO()):
                result = assessment.main(['--manifest', str(Path(directory)/'missing.json'), '--report', str(output)])
            self.assertEqual(result, 0)
            self.assertEqual(json.loads(output.read_text())['status'], 'INCOMPLETE')


if __name__ == '__main__':
    unittest.main()
