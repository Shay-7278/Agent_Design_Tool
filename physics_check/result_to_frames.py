"""Convert physics_result.json into a brick pose list for Gazebo.

    uv run physics_check/result_to_frames.py physics_result.json [frames.json] [--fitted]

Writes a JSON list with one entry per brick, in build order (course 0 first,
then along each course):

    [{"x": ..., "y": ..., "z": ..., "yaw": ...}, ...]

* x, y, z: brick centre in metres (Gazebo / SDF units), world axes, Z up.
* yaw: rotation about Z in radians, in (-pi, pi], measured from world X to
  the brick's long axis (brickL).  A Gazebo box of size (brickL, brickW,
  brickH) has its length along its own X, so this yaw lines it up.  A brick
  turned by pi is the same brick, so yaw is only defined up to pi.

Bricks dropped by the L0 repair are not in the result's brick list and are
not written.  Half bricks are written like full ones (their length is not in
this file).  ``--fitted`` writes the fitted poses (before the repair) instead
of the final ones.  The default output is brick_frames.json next to the
result file.  Plain Python, no Rhino or compas needed.  Exit code 0 when the
result passed L0 and L1, 1 otherwise (the file is written either way).
"""

from __future__ import print_function

import json
import math
import os
import sys

MM = 0.001  # mm -> m


def pose(frame):
    """[origin, xaxis, yaxis] in mm (frame Y = brick length) -> {x, y, z, yaw}."""
    (ox, oy, oz), _xaxis, (tx, ty, _tz) = frame
    return {"x": ox * MM, "y": oy * MM, "z": oz * MM, "yaw": math.atan2(ty, tx)}


def convert(result, fitted=False):
    """List of {x, y, z, yaw} (m, rad), one per brick in build order."""
    key = "fitted_frame" if fitted else "frame"
    return [pose(b[key]) for b in result.get("bricks", [])]


def main(argv):
    import argparse

    ap = argparse.ArgumentParser(description="physics_result.json -> brick poses (x, y, z, yaw) "
                                             "for Gazebo")
    ap.add_argument("result", help="physics_result.json written by physics_check.py")
    ap.add_argument("out", nargs="?", help="output file (default: brick_frames.json next to "
                                           "the result)")
    ap.add_argument("--fitted", action="store_true",
                    help="write the fitted poses (before the repair) instead of the final ones")
    args = ap.parse_args(argv)

    with open(args.result) as fh:
        result = json.load(fh)
    out = args.out or os.path.join(os.path.dirname(os.path.abspath(args.result)),
                                   "brick_frames.json")
    data = convert(result, args.fitted)
    with open(out, "w") as fh:
        json.dump(data, fh, indent=1)

    print("wrote {} brick poses to {}".format(len(data), out))
    dropped = result.get("dropped", [])
    if dropped:
        print("not written: {} brick(s) dropped by the L0 repair".format(len(dropped)))
    ok = bool(result.get("ok"))
    if not ok:
        print("note: this design did not pass the check (L0 {}, L1 {})".format(
            "pass" if result.get("l0_pass") else "fail", result.get("l1_state", "?")))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
