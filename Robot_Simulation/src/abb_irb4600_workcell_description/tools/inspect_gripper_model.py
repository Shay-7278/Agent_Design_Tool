"""Write a reproducible inventory of the SCHUNK gripper Rhino model."""

from __future__ import annotations

import json
import os
import sys
import traceback

import Rhino


package_directory = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
model_path = os.path.join(
    package_directory,
    "meshes",
    "visual",
    "end_effectors",
    "250925_IRB4600_BrickGripperPGF100_modified.3dm",
)
output_path = os.path.join(
    package_directory, "reference", "gripper_model_inventory.json"
)


def _log_exception(exception_type, exception, traceback_object):
    with open(output_path + ".error.log", "w", encoding="utf-8") as stream:
        stream.write("".join(traceback.format_exception(
            exception_type, exception, traceback_object
        )))


sys.excepthook = _log_exception

model = Rhino.FileIO.File3dm.Read(model_path)
if model is None:
    raise RuntimeError(f"Could not read {model_path}")

objects = []
for model_object in model.Objects:
    geometry = model_object.Geometry
    bounds = geometry.GetBoundingBox(True)
    record = {
        "id": str(model_object.Attributes.Id),
        "name": model_object.Attributes.Name or "",
        "layer": model.Layers[model_object.Attributes.LayerIndex].FullPath,
        "type": type(geometry).__name__,
        "bounds": {
            "min": [bounds.Min.X, bounds.Min.Y, bounds.Min.Z],
            "max": [bounds.Max.X, bounds.Max.Y, bounds.Max.Z],
        },
    }
    if isinstance(geometry, Rhino.Geometry.Point):
        record["point"] = [geometry.Location.X, geometry.Location.Y, geometry.Location.Z]
    if isinstance(geometry, Rhino.Geometry.InstanceReferenceGeometry):
        transform = geometry.Xform
        record["instance_transform"] = [
            [transform[row, column] for column in range(4)] for row in range(4)
        ]
    objects.append(record)

inventory = {
    "source": os.path.basename(model_path),
    "units": str(model.Settings.ModelUnitSystem),
    "layers": [layer.FullPath for layer in model.Layers],
    "objects": objects,
}
os.makedirs(os.path.dirname(output_path), exist_ok=True)
with open(output_path, "w", encoding="utf-8") as stream:
    json.dump(inventory, stream, indent=2)
    stream.write("\n")
print(f"Wrote {len(objects)} gripper objects to {output_path}")
