"""Exports brick boxes to a JSON pose list for Gazebo, with each brick's length.

Like ``export_frames_json.py``, but reads the brick boxes (the ``Bricks``
output of ``physics_check_gh.py``) instead of the planes, so each entry also
carries the brick's own length and half bricks can be spawned at half length.
One entry per brick, in the order the boxes come in (connect them in build
order: course 0 first, then along each course):

    [{"x": ..., "y": ..., "z": ..., "yaw": ..., "length": ...}, ...]

* x, y, z: brick centre in metres (Gazebo / SDF units), world axes, Z up.
  The Rhino model is assumed to be in mm.
* yaw: rotation about Z in radians, in (-pi, pi], from world X to the brick's
  long axis (the box plane's Y axis).
* length: the brick's length along that axis in metres (brickL = 0.190 for a
  full brick, 0.095 for a half bat).  In Gazebo, spawn a box of size
  (length, brickW, brickH) with this yaw: an SDF box has its first size along
  its own X, which the yaw lines up with the brick's long axis.

Rhino runs on Windows, so ``path`` is a Windows path (e.g.
``C:\\Users\\me\\frames.json``); from WSL the same file is
``/mnt/c/Users/me/frames.json``.

Inputs:
    bricks:      Brick boxes [mm] (``Bricks`` output of physics_check_gh.py:
                 box plane Y = along the brick, Y interval = its length);
                 DataTrees are flattened.  Set the input's type hint to Box
                 (or leave it as Object)
    path:        Output file path; default ``brick_frames.json`` next to the
                 saved .3dm file
    export:      Boolean toggle - write the file when True
Outputs:
    Path:        Path of the written file (empty if nothing was written)
    Count:       Number of poses written
    Message:     Status / error message
"""

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


def as_box(item):
    """item as a Rhino.Geometry.Box, or None.  Accepts a Box or its GH goo."""
    if item is None:
        return None
    if hasattr(item, "Value") and not hasattr(item, "Plane"):   # GH_Box goo
        item = item.Value
    if hasattr(item, "Plane") and hasattr(item, "Y") and hasattr(item, "Center"):
        return item
    return None


def box_to_pose(box):
    """Rhino box [mm] -> {x, y, z, length [m], yaw [rad]} (yaw of the plane's Y axis)."""
    c, t = box.Center, box.Plane.YAxis
    return {"x": c.X * MM, "y": c.Y * MM, "z": c.Z * MM, "yaw": math.atan2(t.Y, t.X),
            "length": box.Y.Length * MM}


if __name__ == "__main__":
    import Rhino

    bricks = globals().get("bricks", [])
    path = globals().get("path", None)
    export = globals().get("export", False)

    Path = ""
    Count = 0
    Message = "Set export to True to write the file"

    items = [it for it in flatten(bricks) if it is not None]
    boxes = [as_box(it) for it in items]
    bad = sum(1 for b in boxes if b is None)
    boxes = [b for b in boxes if b is not None]

    if not path:
        doc_path = Rhino.RhinoDoc.ActiveDoc.Path
        if doc_path:
            path = os.path.join(os.path.dirname(doc_path), "brick_frames.json")

    if bad:
        Message = ("{} input(s) are not boxes; connect the Bricks output and set the input's "
                   "type hint to Box".format(bad))
    elif not boxes:
        Message = "No bricks connected"
    elif not path:
        Message = "No path given and the .3dm is not saved yet"
    elif export:
        data = [box_to_pose(b) for b in boxes]
        with open(path, "w") as fh:
            json.dump(data, fh, indent=1)
        Path = path
        Count = len(boxes)
        full = max(d["length"] for d in data)
        half = sum(1 for d in data if d["length"] < 0.99 * full)
        Message = "Wrote {} brick poses ({} shorter than a full brick) to {}".format(
            Count, half, path)
