import json
import Rhino
import scriptcontext as sc

path = r"C:\Users\ashle\Desktop\Agent_Design_Tool\Robot_Simulation\src\abb_irb4600_workcell_description\meshes\visual\end_effectors\250925_IRB4600_BrickGripperPGF100_modified.3dm"
out = r"C:\Users\ashle\Desktop\Agent_Design_Tool\Robot_Simulation\src\abb_irb4600_workcell_description\reference\gripper_cad_inventory.json"
doc = Rhino.RhinoDoc.Open(path, Rhino.RhinoDoc.OpenFileOptions())
if not doc:
    raise RuntimeError("failed to open gripper CAD")
items = []
for obj in doc.Objects:
    bb = obj.Geometry.GetBoundingBox(True)
    layer = doc.Layers[obj.Attributes.LayerIndex]
    items.append({
        "id": str(obj.Id),
        "name": obj.Attributes.Name or "",
        "layer": layer.FullPath if layer else "",
        "type": obj.Geometry.ObjectType.ToString(),
        "bbox_min": [bb.Min.X, bb.Min.Y, bb.Min.Z],
        "bbox_max": [bb.Max.X, bb.Max.Y, bb.Max.Z],
        "user_text": {str(k): str(v) for k, v in obj.Attributes.GetUserStrings()},
    })
with open(out, "w", encoding="utf-8") as stream:
    json.dump({"source": path, "objects": items}, stream, indent=2)
doc.Close()
print("wrote " + out)
