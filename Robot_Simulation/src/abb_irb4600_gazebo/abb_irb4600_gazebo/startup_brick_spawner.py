"""Spawn the initial 50-slot pickup jig after the Gazebo workcell exists."""
from __future__ import annotations

import argparse
import json
import subprocess
import time
from pathlib import Path

from .motion import Pose


def _spawn(world: str, name: str, sdf: Path, pose: Pose) -> None:
    x, y, z = pose.position
    qx, qy, qz, qw = pose.orientation_xyzw
    request = (f'sdf_filename: "{sdf}" name: "{name}" pose: {{position: {{x: {x} y: {y} z: {z}}} '
               f'orientation: {{x: {qx} y: {qy} z: {qz} w: {qw}}}}}')
    cmd = ["gz", "service", "-s", f"/world/{world}/create", "--reqtype", "gz.msgs.EntityFactory",
           "--reptype", "gz.msgs.Boolean", "--timeout", "5000", "--req", request]
    result = subprocess.run(cmd, text=True, capture_output=True)
    if result.returncode or "data: true" not in result.stdout.lower():
        raise RuntimeError(f"failed to spawn {name}: {result.stderr or result.stdout}")


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
    placement_doc = json.loads(Path(args.placement_file).read_text(encoding="utf-8"))
    lengths = placement_doc.get("lengths", [])
    if not lengths and isinstance(placement_doc.get("frames"), list):
        lengths = [190.0] * len(placement_doc["frames"])
    if len(pickup) == 0:
        raise ValueError("pickup layout contains no slots")
    model_dir = Path(args.model_dir)
    for index, item in enumerate(pickup):
        pose = _pose(item)
        length = float(lengths[index]) if index < len(lengths) else 190.0
        variant = "brick_half" if abs(length - 95.0) < 1e-6 else "brick"
        _spawn(args.world, f"pickup_brick_{index:03d}", model_dir / variant / "model.sdf", pose)
    print(f"spawned {len(pickup)} pickup bricks")


if __name__ == "__main__":
    main()
