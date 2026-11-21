"""Assembly exports world poses and actual model sizes for a separate assessment."""
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

from abb_irb4600_gazebo.execute_one_brick import OneBrickExecutor


class AssemblyManifestTests(unittest.TestCase):
    def executor(self, root, bricks):
        node = OneBrickExecutor.__new__(OneBrickExecutor)
        node.observations = root / 'observations.jsonl'
        node.placement_file = Path('/ros2_ws/inputs/brick_frames_numeric.json')
        node.placement_frame = 'MT_base'
        node.start_index = 10
        node.world = SimpleNamespace(static_wall_manifest=Mock(return_value=bricks))
        return node

    def test_manifest_keeps_world_poses_and_full_half_geometry(self):
        bricks = [
            {'name': 'pickup_brick_007', 'position': [1.0677, 4.7147, .39995],
             'orientation_xyzw': [0., 0., -.4, .916515138991168],
             'model_sdf': '/models/brick/model.sdf', 'dimensions_m': [.190, .090, .057]},
            {'name': 'pickup_brick_008', 'position': [-.3689, 4.6441, .45695],
             'orientation_xyzw': [0., 0., -.45, .893028554974588],
             'model_sdf': '/models/brick_half/model.sdf', 'dimensions_m': [.095, .090, .057]},
        ]
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            node = self.executor(root, bricks)
            node._export_assembly_manifest()
            result = json.loads((root / 'assembly_manifest.json').read_text())
        self.assertEqual(result['frame'], 'workcell_base')
        self.assertEqual(result['mode'], 'assembly_visualization')
        self.assertEqual(result['placement_frame'], 'MT_base')
        self.assertEqual(result['bricks'], bricks)
        self.assertEqual(result['placement_file'], str(node.placement_file))

    def test_manifest_snapshot_replaces_old_content_and_keeps_prefix(self):
        prefix = {'name': 'pickup_brick_008', 'position': [0., 1., .1],
                  'orientation_xyzw': [0., 0., 0., 1.],
                  'model_sdf': '/models/brick_half/model.sdf', 'dimensions_m': [.095, .09, .057]}
        placed = {'name': 'pickup_brick_010', 'position': [.2, 1., .1],
                  'orientation_xyzw': [0., 0., 0., 1.],
                  'model_sdf': '/models/brick/model.sdf', 'dimensions_m': [.19, .09, .057]}
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            node = self.executor(root, [prefix])
            node._export_assembly_manifest()
            node.world.static_wall_manifest.return_value = [prefix, placed]
            node._export_assembly_manifest()
            result = json.loads((root / 'assembly_manifest.json').read_text())
        self.assertEqual([record['name'] for record in result['bricks']],
                         ['pickup_brick_008', 'pickup_brick_010'])
        self.assertEqual(len(result['bricks']), 2)


if __name__ == '__main__':
    unittest.main()
