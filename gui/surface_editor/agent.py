"""Portable bounded tool loop and public tool descriptions."""
import json

SYSTEM = """You are a conversational surface-design assistant inside a NURBS wall editor.
Use tools for requested edits, not Python code or imagined changes. You can compose general
deformations and explicit control-point positions; wave is only one available operation.
Read the latest surface state before editing. Use its revision in edit_surface/undo_edit.
Manual changes may happen while you think: on revision conflict, inspect again and reassess.
Coordinates are mm. Local vectors are [width, depth, height], fixed when the file was loaded.
Positive depth follows the reported depth vector; do not assume it means towards the camera.
UV coordinates refer to control-grid indices, not physical distance or surface arc length.
Use one edit_surface batch per design change so undo is predictable. Changes are additive
to the current cage; do not claim a parametric modifier history. For 'half as strong', use
the prior operation's actual displacement or inspect coordinates; do not add another wave.
Smooth and wave operate on the cage: evaluated NURBS shape/amplitude is approximate.
Use rebuild_surface(mode='refine') to add points without changing the shape. Use mode='rebuild'
to reduce counts or create a uniform cubic grid, respecting the user's tolerance; report
the sampled deviation. Read state again after topology changes before editing point indices.
Use new_flat_surface to replace the design with a vertical planar wall (undoable).
No trimming, holes or arbitrary new topology.
For unrepresentable requests, explain the limit and suggest a supported approximation.
If orientation or a materially important dimension is ambiguous, ask a short question.
Never claim stability from appearance. Analyze only when requested or needed to answer an
analysis question. analyze_wall starts an asynchronous job: do not poll in this turn.
Completion appears in chat and a follow-up can inspect results. Treat unknown/outdated
results as historical. Separate L0 geometry, L1 equilibrium, and construction stages;
not checked is not failed. Do not infer a causal explanation from correlations alone.
Only report executed edits after a successful tool result. No file saving is exposed to you.
"""


def obj(properties=None, required=None):
    return {"type": "object", "properties": properties or {}, "required": required or [], "additionalProperties": False}


NUM = {"type": "number"}
VEC = {"type": "array", "items": NUM, "minItems": 3, "maxItems": 3}
OP = obj({
    "kind": {"type": "string", "enum": ["translate", "scale", "rotate", "twist", "bend", "taper", "pull", "smooth", "flatten", "wave", "set_points"]},
    "axis": {"type": "string", "enum": ["width", "depth", "height"]},
    "vector": VEC, "factors": VEC, "angle_degrees": NUM, "amount_mm": NUM,
    "factor": NUM, "cycles": NUM, "phase_degrees": NUM,
    "center_uv": {"type": "array", "items": NUM, "minItems": 2, "maxItems": 2},
    "radius_uv": NUM, "strength": NUM, "iterations": {"type": "integer"},
    "keep_boundary": {"type": "boolean"},
    "points": {"type": "array", "items": obj({"i": {"type": "integer"}, "j": {"type": "integer"}, "position": VEC}, ["i", "j", "position"])},
}, ["kind"])

TOOLS = [
    {"name": "rebuild_surface", "description": "Change U/V control-point counts (4..20 each), undoable. refine inserts knots without changing shape and can only increase counts. rebuild fits a uniform cubic grid and rejects sampled deviations above tolerance_mm. Re-inspect state afterwards. Tolerance is a sampled parametric bound, not a certified global maximum.",
     "parameters": obj({"expected_revision": {"type": "integer"}, "count_u": {"type": "integer", "minimum": 4, "maximum": 20}, "count_v": {"type": "integer", "minimum": 4, "maximum": 20}, "mode": {"type": "string", "enum": ["refine", "rebuild"]}, "tolerance_mm": {"type": "number", "minimum": 0, "maximum": 1000}}, ["expected_revision", "count_u", "count_v", "mode", "tolerance_mm"])},
    {"name": "new_flat_surface", "description": "Replace current design with an undoable vertical flat wall in world XZ, starting at origin. Existing wall overlay is hidden. Width and height in mm.",
     "parameters": obj({"expected_revision": {"type": "integer"}, "width_mm": {"type": "number", "minimum": 100, "maximum": 100000}, "height_mm": {"type": "number", "minimum": 100, "maximum": 100000}, "count_u": {"type": "integer", "minimum": 4, "maximum": 20}, "count_v": {"type": "integer", "minimum": 4, "maximum": 20}}, ["expected_revision", "width_mm", "height_mm", "count_u", "count_v"])},
    {"name": "redo_edit", "description": "Redo the latest undone surface edit, including rebuilding.", "parameters": obj({"expected_revision": {"type": "integer"}}, ["expected_revision"])},
    {"name": "get_surface_state", "description": "Current revision, local axes, world control points, bounds and analysis freshness.", "parameters": obj()},
    {"name": "edit_surface", "description": "Atomic batch on current cage. translate(vector); scale(factors); rotate(axis,angle_degrees) about centroid; twist(axis,angle_degrees) from low=0 to high=angle; bend(axis=width|height,amount_mm) adds parabolic depth bulge zero at ends; taper(axis,factor) scales transverse dimensions from low=1 to high=factor; pull(center_uv,radius_uv,vector) Gaussian displacement; smooth(strength=0..1,iterations=1..20) interior neighbor averaging; flatten(axis,strength=0..1) removes that coordinate about centroid (use depth for planar wall); wave(axis=width|height,amount_mm,cycles,phase_degrees) adds approximate depth wave; set_points(points=[{i,j,position}]) sets WORLD positions. Optional keep_boundary preserves outside rows/columns. All other vectors are LOCAL [width,depth,height]. Only use fields belonging to the chosen kind.",
     "parameters": obj({"expected_revision": {"type": "integer"}, "operations": {"type": "array", "items": OP, "minItems": 1, "maxItems": 20}}, ["expected_revision", "operations"])},
    {"name": "undo_edit", "description": "Undo the latest manual or AI change; inspect state first.", "parameters": obj({"expected_revision": {"type": "integer"}}, ["expected_revision"])},
    {"name": "analyze_wall", "description": "Start analysis of a frozen surface revision. fast checks geometry only; full also runs equilibrium and configured construction stages. Returns running; do not poll in this turn.", "parameters": obj({"mode": {"type": "string", "enum": ["fast", "full"]}}, ["mode"])},
    {"name": "get_analysis_summary", "description": "Read analysis freshness, separate check states, displacement, flagged interfaces and construction stages.", "parameters": obj()},
]


def run_agent(provider, history, text, execute, cancelled, emit):
    """Network work runs off the GUI thread; execute bridges back to that thread."""
    provider.start(history[-24:] + [{"role": "user", "content": text}])
    audit = []
    for _ in range(8):
        if cancelled.is_set():
            return "Stopped. Completed edits remain available to undo.", audit
        reply = provider.request(SYSTEM, TOOLS)
        if cancelled.is_set():
            return "Stopped. Completed edits remain available to undo.", audit
        if not reply.calls:
            return reply.text, audit
        if len(reply.calls) > 20:
            raise ValueError("AI requested too many tools in one response")
        results = []
        for call in reply.calls:
            if cancelled.is_set():
                return "Stopped. Completed edits remain available to undo.", audit
            try:
                result = execute(call.name, call.arguments)
            except Exception as exc:
                result = {"error": str(exc)}
            record = {"tool": call.name, "arguments": call.arguments, "result": result}
            audit.append(record)
            emit(record)
            results.append(result)
        provider.accept(reply, results)
    return "Reached the tool-step limit. Completed edits are retained; you can undo or continue.", audit


def portable_turn(text, answer, audit):
    # The next provider receives actual execution evidence, not only prose claims.
    return [{"role": "user", "content": text},
            {"role": "assistant", "content": answer + ("\nTool execution record:\n" + json.dumps(audit) if audit else "")}]
