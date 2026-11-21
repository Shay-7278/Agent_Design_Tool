"""Spawn the initial 50-slot pickup jig after the Gazebo workcell exists."""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from .motion import Pose
from .brick_lifecycle import load_placements
from .brick_models import load_brick_models, pickup_pose_on_platform
from .gz_brick_world import GazeboBrickWorld


def _spawn(adapter: GazeboBrickWorld, name: str, sdf: Path, pose: Pose) -> None:
    pose = pickup_pose_on_platform(pose, sdf)
    adapter.brick_models[name] = sdf
    adapter.spawn(name, pose)


def _pose(item: dict) -> Pose:
    return Pose(tuple(item["position"]), tuple(item["orientation"]))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pickup-file", required=True)
    parser.add_argument("--placement-file", required=True)
    parser.add_argument("--model-dir", required=True)
    parser.add_argument("--world", default="irb4600_workcell")
    parser.add_argument("--startup-delay", type=float, default=3.0)
    args, _ros_args = parser.parse_known_args()
    time.sleep(args.startup_delay)
    pickup = json.loads(Path(args.pickup_file).read_text(encoding="utf-8"))["pickup_bricks"]
    if len(pickup) == 0:
        raise ValueError("pickup layout contains no slots")
    model_dir = Path(args.model_dir)
    brick_models = load_brick_models(args.placement_file, model_dir)
    count = min(len(pickup), len(load_placements(args.placement_file)))
    adapter = GazeboBrickWorld(args.world, model_dir / 'brick/model.sdf', static_supply=True)
    try:
        for index, item in enumerate(pickup[:count]):
            pose = _pose(item)
            _spawn(adapter, f"pickup_brick_{index:03d}", brick_models[index], pose)
    finally:
        adapter.close()
    print(f"spawned {count} pickup bricks")


if __name__ == "__main__":
    main()
