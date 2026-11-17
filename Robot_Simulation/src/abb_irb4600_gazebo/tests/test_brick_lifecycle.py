import json
import tempfile
import unittest
from pathlib import Path

from abb_irb4600_gazebo.brick_lifecycle import (
    BrickLifecycle,
    BrickState,
    Placement,
    BrickPlan,
    load_placements,
)
from abb_irb4600_gazebo.motion import Pose


class FakeWorld:
    def __init__(self):
        self.calls = []

    def spawn(self, name, pose):
        self.calls.append(("spawn", name))

    def attach(self, name):
        self.calls.append(("attach", name))

    def detach(self, name):
        self.calls.append(("detach", name))

    def delete(self, name):
        self.calls.append(("delete", name))


class BrickLifecycleTests(unittest.TestCase):
    def test_runtime_placement_file_is_variable(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "placements.json"
            path.write_text(
                json.dumps([[1,2,3,0]]),
                encoding="utf-8",
            )
            placements = load_placements(path)
        self.assertEqual(placements[0].pose.position, (1.0, 2.0, 3.0))

    def test_lifecycle_requires_ordered_transitions(self):
        pose = Pose((0, 0, 0), (0, 0, 0, 1))
        plan = BrickPlan('pickup_brick_000', pose, pose)
        world = FakeWorld()
        lifecycle = BrickLifecycle(world, [plan])
        lifecycle.spawn_available(plan)
        lifecycle.attach(plan)
        lifecycle.begin_transport(plan)
        lifecycle.release(plan)
        lifecycle.begin_settling(plan)
        lifecycle.mark_placed(plan)
        self.assertEqual(lifecycle.state(plan), BrickState.PLACED)
        self.assertEqual([call[0] for call in world.calls], ["spawn", "attach", "detach"])


if __name__ == "__main__":
    unittest.main()
