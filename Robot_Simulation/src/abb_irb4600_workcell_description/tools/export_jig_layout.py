"""Export the fixed pickup-brick layout referenced by the Grasshopper graph.

Run this script with Rhino 8's Python runtime.  The Grasshopper archive stores
50 Rhino object GUIDs, not copies of the brick geometry, so the matching
workcell .3dm file is required.

Environment variables:
    ARC574_WORKCELL_3DM  matching Rhino workcell file
    ARC574_GRASSHOPPER_GH  Grasshopper graph containing the jig references
    ARC574_JIG_OUTPUT  destination JSON file
"""

from __future__ import annotations

import json
import math
import os
import tempfile
import traceback
import xml.etree.ElementTree as ET

import Rhino
import System

System.Reflection.Assembly.LoadFrom(
    r"C:\Program Files\Rhino 8\Plug-ins\Grasshopper\GH_IO.dll"
)
from GH_IO.Serialization import GH_Archive  # type: ignore  # noqa: E402


def _required_path(variable: str, default: str) -> str:
    path = os.environ.get(variable, default)
    if not path:
        raise RuntimeError(f"{variable} is required")
    path = os.path.abspath(path)
    if variable != "ARC574_JIG_OUTPUT" and not os.path.isfile(path):
        raise RuntimeError(f"{variable} does not exist: {path}")
    return path


def _item_text(node: ET.Element, name: str) -> str | None:
    item = node.find(f".//item[@name='{name}']")
    return item.text.strip() if item is not None and item.text else None


def referenced_jig_ids(graph_path: str) -> list[System.Guid]:
    archive = GH_Archive()
    if not archive.ReadFromFile(graph_path):
        raise RuntimeError(f"Grasshopper archive could not be read: {graph_path}")
    root = ET.fromstring(archive.Serialize_Xml())
    objects = root.findall(".//chunk[@name='Object']")

    jig_group = next(
        (
            node
            for node in objects
            if _item_text(node, "NickName") == "JIG BRICKS (Internalized)"
        ),
        None,
    )
    if jig_group is None:
        raise RuntimeError("JIG BRICKS (Internalized) group was not found")

    member_ids = {
        item.text.strip()
        for item in jig_group.findall(".//item[@name='ID']")
        if item.text
    }
    candidates = [
        node
        for node in objects
        if _item_text(node, "InstanceGuid") in member_ids
        and node.find(".//item[@name='RefID']") is not None
    ]
    if len(candidates) != 1:
        raise RuntimeError(
            f"Expected one referenced jig geometry parameter; found {len(candidates)}"
        )

    ids = [
        System.Guid(item.text.strip())
        for item in candidates[0].findall(".//item[@name='RefID']")
        if item.text
    ]
    if not ids:
        raise RuntimeError("The jig parameter contains no Rhino object references")
    return ids


def _dot(a: Rhino.Geometry.Vector3d, b: Rhino.Geometry.Vector3d) -> float:
    return Rhino.Geometry.Vector3d.Multiply(a, b)


def _unit(vector: Rhino.Geometry.Vector3d) -> Rhino.Geometry.Vector3d:
    result = Rhino.Geometry.Vector3d(vector)
    if not result.Unitize():
        raise RuntimeError("Encountered a zero-length brick edge")
    return result


def _canonical_sign(vector: Rhino.Geometry.Vector3d) -> Rhino.Geometry.Vector3d:
    components = (vector.X, vector.Y, vector.Z)
    dominant = max(range(3), key=lambda index: abs(components[index]))
    return -vector if components[dominant] < 0.0 else vector


def _brick_axes(brep: Rhino.Geometry.Brep) -> tuple[
    Rhino.Geometry.Vector3d,
    Rhino.Geometry.Vector3d,
    Rhino.Geometry.Vector3d,
]:
    directions: list[tuple[Rhino.Geometry.Vector3d, float]] = []
    for edge in brep.Edges:
        start = edge.PointAtStart
        end = edge.PointAtEnd
        vector = end - start
        length = vector.Length
        if length <= 1e-8:
            continue
        unit = _unit(vector)
        match = next(
            (
                index
                for index, (known, _) in enumerate(directions)
                if abs(_dot(unit, known)) > 0.999
            ),
            None,
        )
        if match is None:
            directions.append((_canonical_sign(unit), length))
        elif length > directions[match][1]:
            directions[match] = (directions[match][0], length)

    if len(directions) != 3:
        raise RuntimeError(
            f"Expected three orthogonal brick edge directions; found {len(directions)}"
        )
    world_z = Rhino.Geometry.Vector3d.ZAxis
    z_index = max(range(3), key=lambda index: abs(_dot(directions[index][0], world_z)))
    z_axis = directions[z_index][0]
    if _dot(z_axis, world_z) < 0.0:
        z_axis = -z_axis

    horizontal = [item for index, item in enumerate(directions) if index != z_index]
    x_axis = max(horizontal, key=lambda item: item[1])[0]
    x_axis = _canonical_sign(x_axis)
    y_axis = Rhino.Geometry.Vector3d.CrossProduct(z_axis, x_axis)
    y_axis = _unit(y_axis)
    x_axis = _unit(Rhino.Geometry.Vector3d.CrossProduct(y_axis, z_axis))
    return x_axis, y_axis, z_axis


def _quaternion_xyzw(
    x_axis: Rhino.Geometry.Vector3d,
    y_axis: Rhino.Geometry.Vector3d,
    z_axis: Rhino.Geometry.Vector3d,
) -> list[float]:
    quaternion = Rhino.Geometry.Quaternion()
    target_plane = Rhino.Geometry.Plane(Rhino.Geometry.Point3d.Origin, x_axis, y_axis)
    quaternion.SetRotation(Rhino.Geometry.Plane.WorldXY, target_plane)
    magnitude = math.sqrt(
        quaternion.A**2 + quaternion.B**2 + quaternion.C**2 + quaternion.D**2
    )
    return [
        quaternion.B / magnitude,
        quaternion.C / magnitude,
        quaternion.D / magnitude,
        quaternion.A / magnitude,
    ]


def export_layout(graph_path: str, model_path: str, output_path: str) -> None:
    object_ids = referenced_jig_ids(graph_path)
    model = Rhino.FileIO.File3dm.Read(model_path)
    if model is None:
        raise RuntimeError(f"Rhino workcell file could not be read: {model_path}")
    scale = Rhino.RhinoMath.UnitScale(
        model.Settings.ModelUnitSystem, Rhino.UnitSystem.Meters
    )

    bricks = []
    for index, object_id in enumerate(object_ids):
        model_object = model.Objects.FindId(object_id)
        if model_object is None:
            raise RuntimeError(f"Jig object {object_id} is missing from the workcell file")
        geometry = model_object.Geometry
        if isinstance(geometry, Rhino.Geometry.Extrusion):
            brep = geometry.ToBrep()
        else:
            brep = geometry
        if not isinstance(brep, Rhino.Geometry.Brep):
            raise RuntimeError(
                f"Jig object {object_id} is {type(geometry).__name__}, not box geometry"
            )
        box = brep.GetBoundingBox(True)
        center = box.Center
        x_axis, y_axis, z_axis = _brick_axes(brep)
        bricks.append(
            {
                "id": f"pickup_{index:03d}",
                "position": [center.X * scale, center.Y * scale, center.Z * scale],
                "orientation": _quaternion_xyzw(x_axis, y_axis, z_axis),
                "source_object_id": str(object_id),
            }
        )

    output = {
        "frame": "workcell_base",
        "units": "m",
        "orientation_order": "xyzw",
        "pickup_bricks": bricks,
    }
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as stream:
        json.dump(output, stream, indent=2)
        stream.write("\n")
    print(f"Exported {len(bricks)} fixed pickup poses to {output_path}")


if __name__ == "__main__":
    package_directory = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    default_graph = os.path.join(
        package_directory,
        "ARC574_F26_RoboticBrickAssembly_IRB4600_Rhino8_Template.gh",
    )
    default_model = os.path.join(tempfile.gettempdir(), "arc574_workcell_readonly.3dm")
    default_output = os.path.join(
        package_directory, "config", "fixed_pickup_layout.json"
    )
    output_path = os.environ.get("ARC574_JIG_OUTPUT", default_output)
    try:
        export_layout(
            _required_path("ARC574_GRASSHOPPER_GH", default_graph),
            _required_path("ARC574_WORKCELL_3DM", default_model),
            _required_path("ARC574_JIG_OUTPUT", default_output),
        )
    except Exception:
        with open(output_path + ".error.log", "w", encoding="utf-8") as stream:
            stream.write(traceback.format_exc())
        raise
