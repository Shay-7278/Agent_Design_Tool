import unittest

from abb_irb4600_gazebo.irb4600_kinematics import forward_kinematics, validate_joint_vector, solve_base_target
from abb_irb4600_gazebo.motion import Pose


class IRB4600KinematicsTests(unittest.TestCase):
    def test_base_to_dh_conversion_reproduces_pickup_target(self):
        limits = [(-3.14,3.14),(-1.57,2.61),(-3.14,1.30),(-6.98,6.98),(-2.18,2.09),(-6.98,6.98)]
        for z in (0.307, 0.557):
            target = Pose((1.6018, -0.2439, z), (1,0,0,0))
            joints = solve_base_target(target, limits)
            actual = forward_kinematics(joints)
            for i in range(3):
                self.assertAlmostEqual(actual[i][3], target.position[i], places=7)
            self.assertAlmostEqual(actual[2][2], -1.0, places=7)

    def test_fk_returns_homogeneous_transform(self):
        transform = forward_kinematics([0.0] * 6)
        self.assertEqual(len(transform), 4)
        self.assertEqual(transform[3], (0.0, 0.0, 0.0, 1.0))

    def test_joint_vector_validation(self):
        self.assertTrue(validate_joint_vector([0.0] * 6).reachable)
        self.assertFalse(validate_joint_vector([0.0] * 5).reachable)


if __name__ == "__main__":
    unittest.main()
