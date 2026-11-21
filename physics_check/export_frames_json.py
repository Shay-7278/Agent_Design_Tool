"""Exports brick centre frames to a JSON pose list for Gazebo.

Writes one entry per brick, in the order the frames come in (connect them in
build order: course 0 first, then along each course):

    [{"x": ..., "y": ..., "z": ..., "yaw": ...}, ...]

* x, y, z: brick centre in metres (Gazebo / SDF units), world axes, Z up.
  The Rhino model is assumed to be in mm.
* yaw: rotation about Z in radians, in (-pi, pi], from world X to the brick's
  long axis (the frame's Y axis, brickL).  A Gazebo box of size (brickL,
  brickW, brickH) has its length along its own X, so this yaw lines it up.

Same format as ``result_to_frames.py``.  Half bricks are written like full
ones (lengths are not in this file).

Rhino runs on Windows, so ``path`` is a Windows path (e.g.
``C:\\Users\\me\\frames.json``); from WSL the same file is
``/mnt/c/Users/me/frames.json``.

Inputs:
    frames:      Brick centre frames (planes) [mm], frame Y along the brick
                 length; DataTrees are flattened
    path:        Output file path; default ``brick_frames.json`` next to the
                 saved .3dm file
    export:      Boolean toggle - write the file when True
    (Older canvases may still have brickL / brickW / brickH / mortar_t
    inputs; they are ignored.)
Outputs:
    Path:        Path of the written file (empty if nothing was written)
    Count:       Number of poses written
    Message:     Status / error message
"""

__author__ = "aa9663"
__version__ = "2026.11.15"

import json
import math
import os

MM = 0.001  # mm -> m


def flatten(data):
    """Flatten a DataTree, a [DataTree], or nested lists into one flat list."""
    if hasattr(data, "Branches"):
        items = []
        for b in data.Branches:
            items.extend(list(b))
        return items
    if hasattr(data, "AllData"):
        return list(data.AllData())
    if isinstance(data, (list, tuple)):
        out = []
        for it in data:
            out.extend(flatten(it))
        return out
    return [data]


def plane_to_pose(plane):
    """Rhino plane [mm] -> {x, y, z [m], yaw [rad]} (yaw of the plane's Y axis)."""
    o, t = plane.Origin, plane.YAxis
    return {"x": o.X * MM, "y": o.Y * MM, "z": o.Z * MM, "yaw": math.atan2(t.Y, t.X)}


if __name__ == "__main__":
    import Rhino

    frames = globals().get("frames", [])
    path = globals().get("path", None)
    export = globals().get("export", False)

    Path = ""
    Count = 0
    Message = "Set export to True to write the file"

    planes = [f for f in flatten(frames) if f is not None and hasattr(f, "Origin")]

    if not path:
        doc_path = Rhino.RhinoDoc.ActiveDoc.Path
        if doc_path:
            path = os.path.join(os.path.dirname(doc_path), "brick_frames.json")

    if not planes:
        Message = "No frames connected"
    elif not path:
        Message = "No path given and the .3dm is not saved yet"
    elif export:
        data = [plane_to_pose(p) for p in planes]
        with open(path, "w") as fh:
            json.dump(data, fh, indent=1)
        Path = path
        Count = len(planes)
        Message = "Wrote {} brick poses to {}".format(Count, path)
