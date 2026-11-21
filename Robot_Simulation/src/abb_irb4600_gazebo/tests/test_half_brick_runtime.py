"""The Gazebo supply and refill identities select the confirmed brick models."""
import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from abb_irb4600_gazebo import startup_brick_spawner
from abb_irb4600_gazebo.gz_brick_world import GazeboBrickWorld
from abb_irb4600_gazebo.motion import Pose


HALF_ROWS = (9, 17, 26, 34, 43, 51, 60, 68, 78, 87, 97,
             107, 118, 128, 140, 151, 163, 173, 184, 193, 203, 212)


class HalfBrickRuntimeTests(unittest.TestCase):
    def test_refill_selects_model_by_absolute_brick_identity(self):
        world = GazeboBrickWorld.__new__(GazeboBrickWorld)
        full = Path('/models/brick/model.sdf')
        half = Path('/models/brick_half/model.sdf')
        world.brick_sdf = str(full)
        world.brick_models = {
            f'pickup_brick_{i:03d}': half if i + 1 in HALF_ROWS else full
            for i in range(221)
        }
        world._request = Mock()
        world.models = Mock(return_value={})
        world._confirm = Mock(return_value=108)
        world.pose = Mock()
        pose = Pose((1.2, 2.3, .4), (0, 0, 0, 1))
        for index in (8, 16, 49, 50, 59, 211, 220, 999):
            with self.subTest(source_row=index + 1):
                name = f'pickup_brick_{index:03d}'
                world.spawn(name, pose)
                service, request = world._request.call_args.args
                self.assertEqual(service, 'create')
                self.assertEqual(request.name, name)
                self.assertFalse(request.allow_renaming)
                self.assertEqual(request.sdf_filename,
                                 str(half if index + 1 in HALF_ROWS else full))
                self.assertEqual((request.pose.position.x, request.pose.position.y,
                                  request.pose.position.z), pose.position)
                self.assertIn(name, world._confirm.call_args.args[1])

    def startup(self, total):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        root = Path(temporary.name)
        pickup = root / 'pickup.json'
        placement = root / 'poses.json'
        pickup.write_text(json.dumps({'pickup_bricks': [
            {'position': [i * .22, 0, .0725], 'orientation': [0, 0, 0, 1]}
            for i in range(50)
        ]}), encoding='utf-8')
        placement.write_text(json.dumps([[i * .2, 1, .0285, 0]
                                         for i in range(total)]), encoding='utf-8')
        model_dir = root / 'models'
        models = tuple(model_dir / ('brick_half' if i + 1 in HALF_ROWS else 'brick') / 'model.sdf'
                       for i in range(total))
        arguments = ['spawn_pickup_grid', '--pickup-file', str(pickup),
                     '--placement-file', str(placement), '--model-dir', str(model_dir)]
        with patch('sys.argv', arguments), \
             patch.object(startup_brick_spawner.time, 'sleep'), \
             patch.object(startup_brick_spawner, 'GazeboBrickWorld',
                          return_value=Mock(world_name='irb4600_workcell')), \
             patch.object(startup_brick_spawner, 'load_brick_models', return_value=models) as load, \
             patch.object(startup_brick_spawner, '_spawn') as spawn, \
             contextlib.redirect_stdout(io.StringIO()):
            startup_brick_spawner.main()
        load.assert_called_once()
        self.assertEqual(Path(load.call_args.args[0]), placement)
        self.assertEqual(Path(load.call_args.args[1]), model_dir)
        return spawn.call_args_list

    def test_startup_spawns_the_five_confirmed_halves_in_first_fifty_slots(self):
        calls = self.startup(221)
        self.assertEqual(len(calls), 50)
        halves = []
        for index, call in enumerate(calls):
            world, name, sdf, pose = call.args
            self.assertEqual(name, f'pickup_brick_{index:03d}')
            self.assertEqual(world.world_name, 'irb4600_workcell')
            self.assertEqual(pose.position, (index * .22, 0, .0725))
            self.assertEqual(Path(sdf).parent.name,
                             'brick_half' if index + 1 in HALF_ROWS else 'brick')
            if Path(sdf).parent.name == 'brick_half':
                halves.append(index + 1)
        self.assertEqual(halves, [9, 17, 26, 34, 43])

    def test_short_input_does_not_spawn_unused_supply_slots(self):
        calls = self.startup(17)
        self.assertEqual(len(calls), 17)
        self.assertEqual([i + 1 for i, call in enumerate(calls)
                          if Path(call.args[2]).parent.name == 'brick_half'], [9, 17])


if __name__ == '__main__':
    unittest.main()
