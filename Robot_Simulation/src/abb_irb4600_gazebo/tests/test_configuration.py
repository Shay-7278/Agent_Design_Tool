import unittest
from validate_control_configuration import main as control_check
from validate_brick_model import main as brick_check


class ConfigurationTests(unittest.TestCase):
    def test_installed_controller_and_urdf_agree(self):
        control_check()

    def test_full_brick_physical_parameters(self):
        brick_check()
