import json
import math
import os
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
from construct_using_validated import (
    T_GRASP_IN_BRICK,
    apply_grasp_offset,
    load_poses_file,
    prepare_goal_poses,
)


class LoadPosesFileTests(unittest.TestCase):
    def load_payload(self, payload: dict) -> list[np.ndarray]:
        with tempfile.TemporaryDirectory() as temp_dir:
            pose_file = os.path.join(temp_dir, "poses.json")
            with open(pose_file, "w", encoding="utf-8") as f:
                json.dump(payload, f)
            return load_poses_file(pose_file)

    def test_example_euler_pose_converts_to_xyzw(self) -> None:
        payload = {
            "frame": "world",
            "position_units": "m",
            "orientation_format": "euler_xyz",
            "angle_units": "degrees",
            "bricks": [
                {"position": [0.1, 0.2, 0.03], "orientation": [90, 0, 0]}
            ],
        }

        pose = self.load_payload(payload)[0]
        self.assertEqual(pose[:3].tolist(), [0.1, 0.2, 0.03])
        np.testing.assert_allclose(
            pose[3:],
            [math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)],
            atol=1e-8,
        )

    def test_quaternion_is_normalized_and_preserved_as_xyzw(self) -> None:
        payload = {
            "frame": "world",
            "position_units": "m",
            "orientation_format": "quaternion_xyzw",
            "bricks": [
                {"position": [0, 0, 0.03], "orientation": [0, 0, 0, 2]}
            ],
        }

        pose = self.load_payload(payload)[0]
        np.testing.assert_allclose(pose[3:], [0, 0, 0, 1], atol=1e-8)

    def test_millimetre_positions_convert_to_metres(self) -> None:
        payload = {
            "frame": "world",
            "position_units": "mm",
            "orientation_format": "euler_xyz",
            "angle_units": "degrees",
            "bricks": [
                {"position": [120, 300, 31], "orientation": [0, 0, 0]}
            ],
        }

        pose = self.load_payload(payload)[0]
        np.testing.assert_allclose(pose[:3], [0.12, 0.3, 0.031], atol=1e-10)

    def test_rhino_placement_frame_converts_axes_and_millimetres(self) -> None:
        payload = {
            "offset_placement_frames": [
                {
                    "origin": [100.0, -200.0, 84.69975],
                    "x_axis": [0.0, 1.0, 0.0],
                    "y_axis": [-1.0, 0.0, 0.0],
                    "z_axis": [0.0, 0.0, 1.0],
                }
            ]
        }

        pose = self.load_payload(payload)[0]
        np.testing.assert_allclose(pose[:3], [0.1, -0.2, 0.08469975], atol=1e-10)
        np.testing.assert_allclose(
            np.abs(pose[3:]), [0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)], atol=1e-8
        )

    def test_rhino_frame_rejects_non_orthonormal_axes(self) -> None:
        payload = {
            "offset_placement_frames": [
                {
                    "origin": [0, 0, 0],
                    "x_axis": [1, 0, 0],
                    "y_axis": [1, 0, 0],
                    "z_axis": [0, 0, 1],
                }
            ]
        }

        with self.assertRaisesRegex(ValueError, "orthonormal"):
            self.load_payload(payload)

    def test_attached_rhino_file_loads_all_frames_when_available(self) -> None:
        attached = os.environ.get("ATTACHED_POSES_FILE")
        if not attached:
            self.skipTest("ATTACHED_POSES_FILE is not set")
        poses = load_poses_file(attached)
        self.assertEqual(len(poses), 270)
        np.testing.assert_allclose(poses[0][:3], [-0.2559701152, -0.4209436997, 0.08469975])

    def test_rejects_unknown_coordinate_frame(self) -> None:
        payload = {
            "frame": "grasshopper",
            "position_units": "m",
            "orientation_format": "euler_xyz",
            "angle_units": "degrees",
            "bricks": [
                {"position": [0, 0, 0.03], "orientation": [0, 0, 0]}
            ],
        }

        with self.assertRaisesRegex(ValueError, 'frame.*world'):
            self.load_payload(payload)

    def test_rejects_non_finite_position(self) -> None:
        payload = {
            "frame": "world",
            "position_units": "m",
            "orientation_format": "euler_xyz",
            "angle_units": "degrees",
            "bricks": [
                {"position": [0, float("nan"), 0.03], "orientation": [0, 0, 0]}
            ],
        }

        with self.assertRaisesRegex(ValueError, "non-finite"):
            self.load_payload(payload)

    def test_goal_preparation_limits_to_20_and_places_lowest_layer_on_table(self) -> None:
        goals = [
            np.array([0.1, 0.2, 0.08469975 + i * 0.001, 0.0, 0.0, 0.0, 1.0])
            for i in range(25)
        ]

        prepared = prepare_goal_poses(goals)

        self.assertEqual(len(prepared), 20)
        self.assertAlmostEqual(prepared[0][2], 0.026)
        self.assertAlmostEqual(prepared[19][2], 0.045)
        self.assertAlmostEqual(goals[0][2], 0.08469975)

    def test_fixed_grasp_is_orthogonal_and_points_down_brick_z(self) -> None:
        brick_pose = np.array([0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0])

        tcp_xyz, tcp_quat = apply_grasp_offset(brick_pose, T_GRASP_IN_BRICK)

        np.testing.assert_allclose(tcp_xyz, brick_pose[:3], atol=1e-10)
        np.testing.assert_allclose(np.abs(tcp_quat), [1.0, 0.0, 0.0, 0.0], atol=1e-10)
        np.testing.assert_allclose(T_GRASP_IN_BRICK[:3, 2], [0.0, 0.0, -1.0])


if __name__ == "__main__":
    unittest.main()
