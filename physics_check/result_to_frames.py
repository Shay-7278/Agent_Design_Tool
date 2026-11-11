"""Convert physics_result.json into a brick frames file with per-brick lengths.

    uv run physics_check/result_to_frames.py physics_result.json [frames.json] [--fitted]

Writes the ``export_frames_json.py`` format (units, brickL, brickW, brickH,
mortar_t, frames as [origin, xaxis, yaxis] in mm; X = across the wall
(brickW), Y = along the course (brickL), Z = up) plus:

* ``lengths``: each brick's own length along frame Y [mm], in the order of
  ``frames``.  The half bricks at the ends of odd open courses are brickL / 2;
  the shared ``brickL`` alone would export them at full length, sticking
  past the wall ends into their neighbours.  A reader that does not know this
  key still gets the frames with brickL.
* ``bricks``: course, index and status of each frame, in the same order.

Bricks dropped by the L0 repair are not in the result's brick list and are
not written.  ``--fitted`` writes the fitted frames (before the repair)
instead of the final ones.  The default output is brick_frames.json next to
the result file.  Plain Python, no Rhino or compas needed.  Exit code 0 when
the result passed L0 and L1, 1 otherwise (the file is written either way).
"""

from __future__ import print_function

import json
import os
import sys


def convert(result, fitted=False):
    """Frames dict (export_frames_json format + lengths, bricks) from a result dict."""
    bricks = result.get("bricks", [])
    key = "fitted_frame" if fitted else "frame"
    return {
        "units": result.get("units", "mm"),
        "brickL": float(result["brickL"]),
        "brickW": float(result["brickW"]),
        "brickH": float(result["brickH"]),
        "mortar_t": float(result["mortar_t"]),
        "source": "fitted frames" if fitted else "frames after the checks",
        "frames": [b[key] for b in bricks],
        "lengths": [float(b.get("length", result["brickL"])) for b in bricks],
        "bricks": [{"course": b["course"], "index": b["index"], "status": b["status"]}
                   for b in bricks],
    }


def main(argv):
    import argparse

    ap = argparse.ArgumentParser(description="physics_result.json -> brick frames JSON with "
                                             "per-brick lengths")
    ap.add_argument("result", help="physics_result.json written by physics_check.py")
    ap.add_argument("out", nargs="?", help="output file (default: brick_frames.json next to "
                                           "the result)")
    ap.add_argument("--fitted", action="store_true",
                    help="write the fitted frames (before the repair) instead of the final ones")
    args = ap.parse_args(argv)

    with open(args.result) as fh:
        result = json.load(fh)
    out = args.out or os.path.join(os.path.dirname(os.path.abspath(args.result)),
                                   "brick_frames.json")
    data = convert(result, args.fitted)
    with open(out, "w") as fh:
        json.dump(data, fh, indent=1)

    half = sum(1 for ln in data["lengths"] if abs(ln - data["brickL"]) > 1e-6)
    print("wrote {} frames ({} not brickL long) to {}".format(len(data["frames"]), half, out))
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
