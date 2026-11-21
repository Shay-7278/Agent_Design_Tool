"""Verify half-brick physical scaling independently of the spawn selection."""
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


class HalfBrickModelTests(unittest.TestCase):
    def test_dimensions_density_and_box_inertia(self):
        models = Path(__file__).resolve().parents[1] / 'models'
        half = ET.parse(models / 'brick_half/model.sdf').getroot().find('model')
        full = ET.parse(models / 'brick/model.sdf').getroot().find('model')
        self.assertEqual(half.findtext('static'), 'false')
        link = half.find('link')
        self.assertEqual(link.get('name'), 'brick_link')
        dims = (.095, .090, .057)
        for kind in ('collision', 'visual'):
            self.assertEqual(tuple(map(float, link.findtext(f'{kind}/geometry/box/size').split())), dims)
        mass = float(link.findtext('inertial/mass'))
        self.assertAlmostEqual(mass, 700 * dims[0] * dims[1] * dims[2])
        self.assertAlmostEqual(mass * 2, float(full.findtext('link/inertial/mass')))
        x, y, z = dims
        for axis, expected in zip(('ixx', 'iyy', 'izz'),
                                  (mass*(y*y+z*z)/12, mass*(x*x+z*z)/12, mass*(x*x+y*y)/12)):
            self.assertAlmostEqual(float(link.findtext(f'inertial/inertia/{axis}')), expected, places=15)
        for axis in ('ixy', 'ixz', 'iyz'):
            self.assertEqual(float(link.findtext(f'inertial/inertia/{axis}')), 0.)
        for field in ('friction/ode/mu', 'friction/ode/mu2', 'contact/ode/kp', 'contact/ode/kd',
                      'contact/ode/max_vel', 'contact/ode/min_depth'):
            self.assertEqual(link.findtext(f'collision/surface/{field}'),
                             full.findtext(f'link/collision/surface/{field}'))


if __name__ == '__main__':
    unittest.main()
