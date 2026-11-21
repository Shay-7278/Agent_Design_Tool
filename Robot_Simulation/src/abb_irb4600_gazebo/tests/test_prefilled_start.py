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
        self.static_names = set()
        self.events = []
        self.activated = []

    def models(self):
        return {name: i + 1 for i, name in enumerate(self.poses)
                if name not in self.static_names}

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
        self.events.append(('attach', name))

    def activate(self, name):
        if name in self.static_names:
            raise AssertionError(f'cannot activate a completed wall brick: {name}')
        self.activated.append(name)
        self.events.append(('activate', name))

    def detach(self, name):
        if self._states[name] != 'attached':
            raise AssertionError(f'{name} was not attached')
        self._states[name] = 'detached'
        self.released.append(name)
        self.events.append(('detach', name))

    def consolidate(self, name, pose):
        if self._states.get(name) != 'detached':
            raise AssertionError(f'{name} consolidated before release')
        self.poses[name] = pose
        self.static_names.add(name)
        self._states[name] = 'static'
        self.events.append(('consolidate', name))

    def prefill_static(self, placements):
        for name, pose in placements:
            self.relocated.append(name)
            self.poses[name] = pose
            self.static_names.add(name)
            self.events.append(('prefill', name))


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
            n.world.events.append(('move', n.current_plan.name, target))
            for name, state in n.world._states.items():
                if state == 'attached':
                    n.world.poses[name] = target

        with ExitStack() as stack:
            for method in ('get_logger', '_spin_for', 'send', '_record_placement',
                           '_export_assembly_manifest'):
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
        self.assertEqual(len(n.world.relocated), 60)
        self.assertEqual([name for name, _ in n.world.spawned],
                         [f'pickup_brick_{i:03d}' for i in range(60, 62)])
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

    def test_startup_readiness_waits_for_last_supply_without_mutating_world(self):
        with tempfile.TemporaryDirectory() as directory:
            n = self.executor(directory, start_brick=11, total=221)
            complete = n.world.models()
            partial = dict(complete)
            partial.pop('pickup_brick_049')
            before = dict(n.world.poses)
            n.world.models = Mock(side_effect=[partial, partial, complete])
            with patch('abb_irb4600_gazebo.execute_one_brick.rclpy.spin_once') as spin:
                n._await_startup_batch()
            self.assertEqual(n.world.models.call_count, 3)
            self.assertTrue(spin.called)
            self.assertEqual(n.world.poses, before)
            self.assertEqual(n.world.spawned, [])
            self.assertEqual(n.world.relocated, [])
            self.assertEqual(n.world.activated, [])

    def test_release_is_consolidated_before_retreat_and_never_carried_again(self):
        with tempfile.TemporaryDirectory() as directory:
            n = self.executor(directory, start_brick=1, total=2)
            self.run_executor(n)
        for name in ('pickup_brick_000', 'pickup_brick_001'):
            detach = n.world.events.index(('detach', name))
            freeze = n.world.events.index(('consolidate', name))
            self.assertLess(detach, freeze)
            later_moves = [i for i, event in enumerate(n.world.events)
                           if event[0] == 'move' and event[1] == name and i > detach]
            self.assertTrue(later_moves, 'placement has no retreat/home movement')
            self.assertLess(freeze, min(later_moves))
            self.assertIn(name, n.world.static_names)
        self.assertEqual(n.world.activated, n.world.released)


if __name__ == '__main__':
    unittest.main()
