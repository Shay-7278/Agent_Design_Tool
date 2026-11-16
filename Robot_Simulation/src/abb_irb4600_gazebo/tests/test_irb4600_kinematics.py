import unittest

from abb_irb4600_gazebo.irb4600_kinematics import forward_kinematics, validate_joint_vector


class IRB4600KinematicsTests(unittest.TestCase):
    def test_fk_returns_homogeneous_transform(self):
        transform = forward_kinematics([0.0] * 6)
        self.assertEqual(len(transform), 4)
        self.assertEqual(transform[3], (0.0, 0.0, 0.0, 1.0))

    def test_joint_vector_validation(self):
        self.assertTrue(validate_joint_vector([0.0] * 6).reachable)
        self.assertFalse(validate_joint_vector([0.0] * 5).reachable)


if __name__ == "__main__":
    unittest.main()
