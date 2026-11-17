"""Resume-by-prefill keeps source row identities and pickup-batch boundaries."""
import json
import tempfile
import unittest
from contextlib import ExitStack
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from geometry_msgs.msg import TransformStamped

from abb_irb4600_gazebo.brick_lifecycle import Placement
from abb_irb4600_gazebo.execute_one_brick import ARM_JOINTS, OneBrickExecutor
from abb_irb4600_gazebo.motion import Pose


class PrefillWorld:
    def __init__(self, pickups, count):
        self.poses = {f'pickup_brick_{i:03d}': p for i, p in enumerate(pickups[:count])}
        self._states = {}
        self.relocated = []
        self.spawned = []
        self.released = []

    def models(self):
        return {name: i + 1 for i, name in enumerate(self.poses)}

    def pose(self, name):
        return self.poses[name]

    def observation(self, name, after_sequence=None):
        return self.poses[name], 1 if after_sequence is None else after_sequence + 1, 0.0

    def set_pose(self, name, pose):
        if name not in self.poses:
            raise AssertionError(f'cannot relocate missing model {name}')
        self.relocated.append(name)
        self.poses[name] = pose

    def spawn(self, name, pose):
        if name in self.poses:
            raise AssertionError(f'duplicate model {name}')
        self.spawned.append((name, len(self.released)))
        self.poses[name] = pose

    def attach(self, name):
        self._states[name] = 'attached'

    def detach(self, name):
        if self._states[name] != 'attached':
            raise AssertionError(f'{name} was not attached')
        self._states[name] = 'detached'
        self.released.append(name)


class PrefilledStartTests(unittest.TestCase):
    def executor(self, directory, start_brick, total, preflight_only=False):
        n = OneBrickExecutor.__new__(OneBrickExecutor)
        identity = (0, 0, 0, 1)
        n.start_index = start_brick - 1
        n.pickups = [Pose((i * .22, 0, .1), identity) for i in range(50)]
        n.placements = [Placement(i, Pose((i * .22, 1, .1), identity)) for i in range(total)]
        n.input_count = total
        n.placement_frame = 'MT_base'
        n.preflight_only = preflight_only
        n.observations = Path(directory) / 'observations.jsonl'
        n.profile = {'retract_m': .2, 'wait_s': 1.0, 'track_offset_m': -1.0, 'cfx': 0}
        n.home = Pose((.5, 3.45103, 1), identity)
        n.joints = dict.fromkeys(ARM_JOINTS, 0.0)
        n.tool_cfg = {'finger_travel': .008875}
        n.flange_tcp = Pose((0, 0, .24313), identity)
        n.current_plan = None
        n.gripper = None
        tf = TransformStamped()
        tf.transform.rotation.w = 1.0
        n.tf_buffer = SimpleNamespace(can_transform=lambda *a: True, lookup_transform=lambda *a: tf)
        n.world = PrefillWorld(n.pickups, min(50, total))
        return n

    def run_executor(self, n):
        executed = []

        def move(target, *args):
            executed.append(n.current_plan.name)
            for name, state in n.world._states.items():
                if state == 'attached':
                    n.world.poses[name] = target

        with ExitStack() as stack:
            for method in ('get_logger', '_spin_for', 'send', '_record_placement'):
                stack.enter_context(patch.object(OneBrickExecutor, method, return_value=Mock()))
            stack.enter_context(patch.object(OneBrickExecutor, '_solve', return_value=(0, (0,) * 6)))
            stack.enter_context(patch.object(OneBrickExecutor, '_move_target', side_effect=move))
            planning = stack.enter_context(patch.object(
                OneBrickExecutor, '_plan_motion',
                return_value=([SimpleNamespace(positions=[0] * 7)], .5, (0,) * 6)))
            n.run()
        return executed, planning

    def test_start_11_with_total_cap_11_executes_only_original_row_11(self):
        with tempfile.TemporaryDirectory() as directory:
            n = self.executor(directory, start_brick=11, total=11)
            executed, _ = self.run_executor(n)
            self.assertEqual(set(executed), {'pickup_brick_010'})
            self.assertEqual(n.world.released, ['pickup_brick_010'])
            self.assertEqual(n.current_plan.pickup, n.pickups[10])
            self.assertEqual(n.current_plan.placement, n.placements[10].pose)
            self.assertEqual(n.world.relocated, [f'pickup_brick_{i:03d}' for i in range(10)])
            self.assertEqual(n.world.spawned, [])
            records = [json.loads(line) for line in n.observations.read_text().splitlines()]
            self.assertEqual(len(records), 10)
            self.assertTrue(all(record['state'] == 'PREFILLED' for record in records))

    def test_total_cap_does_not_reject_unused_startup_slots(self):
        with tempfile.TemporaryDirectory() as directory:
            n = self.executor(directory, start_brick=11, total=11)
            # Startup reads the full input, independently of executor --max-bricks.
            n.world = PrefillWorld(n.pickups, 50)
            n.input_count = 221
            untouched = {name: pose for name, pose in n.world.poses.items()
                         if int(name.rsplit('_', 1)[1]) >= 11}
            executed, _ = self.run_executor(n)
        self.assertEqual(set(executed), {'pickup_brick_010'})
        self.assertEqual(n.world.spawned, [])
        for name, pose in untouched.items():
            self.assertEqual(n.world.poses[name], pose)

    def test_start_49_preserves_absolute_refill_boundary_at_brick_51(self):
        with tempfile.TemporaryDirectory() as directory:
            n = self.executor(directory, start_brick=49, total=52)
            executed, _ = self.run_executor(n)
        expected = [f'pickup_brick_{i:03d}' for i in range(48, 52)]
        self.assertEqual(set(executed), set(expected))
        self.assertEqual(n.world.released, expected)
        self.assertEqual(n.world.spawned, [('pickup_brick_050', 2), ('pickup_brick_051', 2)])
        self.assertEqual(n.current_plan.pickup, n.pickups[1])

    def test_start_in_later_batch_seeds_missing_prefix_without_reusing_pickup_models(self):
        with tempfile.TemporaryDirectory() as directory:
            n = self.executor(directory, start_brick=61, total=62)
            executed, _ = self.run_executor(n)
        self.assertEqual(set(executed), {'pickup_brick_060', 'pickup_brick_061'})
        self.assertEqual(n.world.released, ['pickup_brick_060', 'pickup_brick_061'])
        self.assertEqual(len(n.world.poses), 62)
        self.assertEqual(len(n.world.relocated), 50)
        self.assertEqual([name for name, _ in n.world.spawned],
                         [f'pickup_brick_{i:03d}' for i in range(50, 62)])
        self.assertEqual(n.world.poses['pickup_brick_059'], n.placements[59].pose)
        self.assertEqual(n.current_plan.pickup, n.pickups[11])

    def test_preflight_only_has_no_world_mutations_or_prefix_motion_plans(self):
        with tempfile.TemporaryDirectory() as directory:
            n = self.executor(directory, start_brick=11, total=11, preflight_only=True)
            before = dict(n.world.poses)
            executed, planning = self.run_executor(n)
            self.assertFalse(n.observations.exists())
        self.assertEqual(executed, [])
        self.assertEqual(n.world.relocated, [])
        self.assertEqual(n.world.spawned, [])
        self.assertEqual(n.world.poses, before)
        # Row 11's pickup/placement and common home targets are the only endpoints.
        allowed = [n.pickups[10].position, n.placements[10].pose.position, n.home.position]
        allowed += [(x, y, z + .2) for x, y, z in allowed[:2]]
        self.assertTrue(planning.call_count)
        for call in planning.call_args_list:
            self.assertIn(call.args[1].position, allowed)

    def test_dirty_pickup_world_is_rejected_before_any_prefix_relocation(self):
        with tempfile.TemporaryDirectory() as directory:
            n = self.executor(directory, start_brick=11, total=11)
            n.world.poses['pickup_brick_009'] = Pose((99, 99, .1), (0, 0, 0, 1))
            with self.assertRaises(RuntimeError):
                self.run_executor(n)
        self.assertEqual(n.world.relocated, [])
        self.assertEqual(n.world.spawned, [])
        self.assertEqual(n.world.released, [])


if __name__ == '__main__':
    unittest.main()
