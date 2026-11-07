"""Exports brick centre frames to a JSON file for the CRA stability check.

Writes the ``Frames`` from ``brick_corbel_column`` (or ``WallFrames`` from
``brick_wall_populate``) plus the brick dimensions to a JSON file.  The file is
read by ``cra_check.py``, which runs outside Rhino (plain CPython + compas_cra)
and checks whether the bricks stand as a dry-stacked rigid-block assembly.

Each frame is stored as ``[origin, xaxis, yaxis]`` in mm, with X = overhang
(brickW), Y = along the wall (brickL), Z = up, matching the other scripts.

Rhino runs on Windows, so ``path`` is a Windows path (e.g.
``C:\\Users\\me\\frames.json``); from WSL the same file is
``/mnt/c/Users/me/frames.json``.

Inputs:
    frames:      Brick centre frames (planes) [mm]; DataTrees are flattened
    brickL:      Brick length - runs along the wall (frame Y) [mm], default 190
    brickW:      Brick width - overhang direction (frame X) [mm], default 90
    brickH:      Brick height [mm], default 57
    mortar_t:    Mortar bed joint thickness [mm], default 10
    path:        Output file path; default ``brick_frames.json`` next to the
                 saved .3dm file
    export:      Boolean toggle - write the file when True
Outputs:
    Path:        Path of the written file (empty if nothing was written)
    Count:       Number of frames written
    Message:     Status / error message
"""

__author__ = "aa9663"
__version__ = "2023.09.11"

import json
import os


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


def frame_to_list(plane):
    """Rhino plane -> [[ox, oy, oz], [xx, xy, xz], [yx, yy, yz]]."""
    o, x, y = plane.Origin, plane.XAxis, plane.YAxis
    return [[o.X, o.Y, o.Z], [x.X, x.Y, x.Z], [y.X, y.Y, y.Z]]


if __name__ == "__main__":
    import Rhino

    frames = globals().get("frames", [])
    brickL = globals().get("brickL", 190.0)
    brickW = globals().get("brickW", 90.0)
    brickH = globals().get("brickH", 57.0)
    mortar_t = globals().get("mortar_t", 10.0)
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
        data = {
            "units": "mm",
            "brickL": float(brickL),
            "brickW": float(brickW),
            "brickH": float(brickH),
            "mortar_t": float(mortar_t),
            "frames": [frame_to_list(p) for p in planes],
        }
        with open(path, "w") as fh:
            json.dump(data, fh, indent=1)
        Path = path
        Count = len(planes)
        Message = "Wrote {} frames to {}".format(Count, path)
