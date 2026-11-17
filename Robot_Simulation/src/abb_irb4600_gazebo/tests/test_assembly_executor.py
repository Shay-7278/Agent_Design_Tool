"""Exercise real executor batch scheduling with a fake transport, without motion."""
import unittest
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from geometry_msgs.msg import TransformStamped
from ament_index_python.packages import get_package_share_directory
from abb_irb4600_gazebo.execute_one_brick import OneBrickExecutor, ARM_JOINTS
from abb_irb4600_gazebo.motion import Pose
from abb_irb4600_gazebo.brick_lifecycle import Placement


class BatchTests(unittest.TestCase):
    def test_221_bricks_replenish_only_after_each_50_are_placed(self):
        n = OneBrickExecutor.__new__(OneBrickExecutor)
        identity = (0, 0, 0, 1)
        n.pickups = [Pose((i * .22, 0, .1), identity) for i in range(50)]
        n.placements = [Placement(i, Pose((0, 1, .1), identity)) for i in range(221)]
        n.placement_frame = 'MT_base'
        n.preflight_only = False
        n.profile = json.loads((Path(get_package_share_directory('abb_irb4600_gazebo'))/'config/gh_motion_profile.json').read_text())
        n.home = Pose((.5,3.45103,1), identity)
        n.joints = dict.fromkeys(ARM_JOINTS, 0.0)
        n.tool_cfg = {'finger_travel': .008875}
        n.flange_tcp = Pose((0,0,.24313), identity)
        tf = TransformStamped()
        tf.transform.rotation.w = 1.0
        n.tf_buffer = SimpleNamespace(can_transform=lambda *a: True, lookup_transform=lambda *a: tf)
        poses = {f'pickup_brick_{i:03d}': p for i,p in enumerate(n.pickups)}
        states = {}
        spawned = []
        released = []

        def spawn(name, pose):
            self.assertNotIn(name, poses)
            spawned.append((name, len(released)))
            poses[name] = pose

        def move(target, *args):
            for name,state in states.items():
                if state == 'attached':
                    poses[name] = target

        def detach(name):
            self.assertEqual(states[name], 'attached')
            states[name] = 'detached'
            released.append(name)

        n.world = SimpleNamespace(pose=lambda name: poses[name], spawn=spawn,
                                  attach=lambda name: states.update({name:'attached'}),
                                  detach=detach, _states=states)
        with patch.object(OneBrickExecutor, 'get_logger', return_value=Mock()), \
             patch.object(OneBrickExecutor, '_solve', return_value=(0, (0,)*6)), \
             patch.object(OneBrickExecutor, '_spin_for'), \
             patch.object(OneBrickExecutor, 'send'), \
             patch.object(OneBrickExecutor, '_move_target', side_effect=move), \
             patch.object(OneBrickExecutor, '_plan_motion', return_value=([SimpleNamespace(positions=[0]*7)],.5,(0,)*6)), \
             patch.object(OneBrickExecutor, '_record_placement'):
            n.gripper = None
            n.run()
        self.assertEqual(len(released), 221)
        self.assertEqual(len(poses), 221)
        self.assertEqual(len(spawned), 171)
        for name, released_count in spawned:
            index = int(name.rsplit('_', 1)[1])
            self.assertEqual(released_count, index // 50 * 50)
        self.assertEqual(len(set(released)), 221)


if __name__ == '__main__':
    unittest.main()
