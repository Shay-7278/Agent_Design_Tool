"""Check the direct-mount CAD datum against the generated workcell and planner."""
import math
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

from abb_irb4600_gazebo.motion import Pose
from abb_irb4600_gazebo.target_motion import (
    flange_target_from_tcp,
    quaternion_rotate,
    rpy_to_quaternion,
    tcp_target_from_brick,
)


class GripperFramesTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        share = Path(get_package_share_directory("abb_irb4600_workcell_description"))
        cls.cfg = yaml.safe_load((share / "config/tools.yaml").read_text())["tools"]["schunk_pgf_100"]
        cls.robot = ET.fromstring(xacro.process_file(str(share / "urdf/abb_irb4600_workcell.xacro")).toxml())

    def test_cad_pad_midpoint_and_mating_plane_match_urdf_tcp(self):
        body = self.robot.find("./joint[@name='LL_tool_attachment_to_schunk_body']/origin")
        offset = tuple(map(float, body.attrib["xyz"].split()))
        rotation = rpy_to_quaternion(*map(float, body.attrib["rpy"].split()))
        for cad_point, expected_z in [((0, 0, 0.04178), 0.0), ((0, 0, -0.20135), 0.24313), ((0, 0, -0.22135), 0.26313)]:
            actual = tuple(a + b for a, b in zip(offset, quaternion_rotate(rotation, cad_point)))
            for value, expected in zip(actual, (0.0, 0.0, expected_z)):
                self.assertAlmostEqual(value, expected, places=9)
        tcp = self.robot.find("./joint[@name='LL_tool_attachment_to_schunk_tcp']/origin")
        self.assertEqual(list(map(float, tcp.attrib["xyz"].split())), self.cfg["flange_tcp"]["xyz"])
        self.assertEqual(list(map(float, tcp.attrib["rpy"].split())), [0.0, 0.0, 0.0])

    def test_downward_pickup_and_tcp_pivot_roundtrip(self):
        cfg = self.cfg["flange_tcp"]
        tool = Pose(tuple(cfg["xyz"]), rpy_to_quaternion(*cfg["rpy"]))
        brick = Pose((1.0, 2.0, 0.0285), (0.0, 0.0, 0.0, 1.0))
        down = tcp_target_from_brick(brick)
        jaw_axis = quaternion_rotate(down.orientation_xyzw, (1.0, 0.0, 0.0))
        self.assertAlmostEqual(jaw_axis[0], 0.0)
        self.assertAlmostEqual(jaw_axis[1], 1.0)
        flange = flange_target_from_tcp(down, tool)
        self.assertAlmostEqual(flange.position[2], 0.0285 + 0.24313)
        for orientation in [down.orientation_xyzw, rpy_to_quaternion(0.4, 0.7, 1.2), rpy_to_quaternion(math.pi, 0.0, 0.8)]:
            target = Pose(brick.position, orientation)
            flange = flange_target_from_tcp(target, tool)
            pad_offset = quaternion_rotate(flange.orientation_xyzw, tool.position)
            for value, expected in zip((a + b for a, b in zip(flange.position, pad_offset)), target.position):
                self.assertAlmostEqual(value, expected, places=9)


if __name__ == "__main__":
    unittest.main()
