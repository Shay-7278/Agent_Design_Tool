"""Portable bounded tool loop and public tool descriptions."""
import json

SYSTEM = """You are a conversational surface-design assistant inside a NURBS wall editor.
Use tools for requested edits, not Python code or imagined changes. You can compose general
deformations and explicit control-point positions; wave is only one available operation.
Read the latest surface state before editing. Use its revision in edit_surface, add_control_points and undo_edit.
Manual changes may happen while you think: on revision conflict, inspect again and reassess.
Coordinates are mm. Local vectors are [width, depth, height], fixed when the file was loaded.
Positive depth follows the reported depth vector; do not assume it means towards the camera.
UV coordinates refer to control-grid indices, not physical distance or surface arc length.
Use one edit_surface batch per design change so undo is predictable. Changes are additive
to the current cage; do not claim a parametric modifier history. For 'half as strong', use
the prior operation's actual displacement or inspect coordinates; do not add another wave.
Grid index i runs along the wall (width), j up it (height). A coarse cage cannot hold fine
detail: before detailed edits such as several waves, call add_control_points in that
direction (aim for at least 5 control points per wave cycle). It keeps the shape exactly and is
undoable, but changes indices, so read the state again afterwards. Smooth and wave operate
on the cage: the evaluated surface moves less than the cage (often 60-80% on a fine grid,
far less on a coarse one), so say so instead of reporting cage offsets as the result. For
undulation at the base only, use wave along width with fade_height (fraction of wall height
over which it fades to zero). No trimming, holes or new topology.
The wall base must stay level: every control point on the base_edge reported by
get_surface_state must keep one shared height, so brick courses stay horizontal and can be
fitted. The rule concerns height only: moving base points in depth or width keeps it level,
and raising or lowering the whole wall is fine; tilting or warping the base vertically is
rejected. keep_boundary pins the base row AND the end columns, so never use it for edits
meant to change the base or the wall ends.
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
    "factor": NUM, "cycles": NUM, "phase_degrees": NUM, "fade_height": NUM,
    "center_uv": {"type": "array", "items": NUM, "minItems": 2, "maxItems": 2},
    "radius_uv": NUM, "strength": NUM, "iterations": {"type": "integer"},
    "keep_boundary": {"type": "boolean"},
    "points": {"type": "array", "items": obj({"i": {"type": "integer"}, "j": {"type": "integer"}, "position": VEC}, ["i", "j", "position"])},
}, ["kind"])

TOOLS = [
    {"name": "get_surface_state", "description": "Current revision, local axes, world control points, bounds and analysis freshness.", "parameters": obj()},
    {"name": "edit_surface", "description": "Atomic batch on current cage. translate(vector); scale(factors); rotate(axis,angle_degrees) about centroid; twist(axis,angle_degrees) from low=0 to high=angle; bend(axis=width|height,amount_mm) adds parabolic depth bulge zero at ends; taper(axis,factor) scales transverse dimensions from low=1 to high=factor; pull(center_uv,radius_uv,vector) Gaussian displacement; smooth(strength=0..1,iterations=1..20) interior neighbor averaging; flatten(axis,strength=0..1) removes that coordinate about centroid (use depth for planar wall); wave(axis=width|height,amount_mm,cycles,phase_degrees,optional fade_height=0.01..1 for width waves: full at base, zero above that fraction of height) adds approximate depth wave, cycles limited by control points along that axis; set_points(points=[{i,j,position}]) sets WORLD positions. Optional keep_boundary preserves all outside rows/columns, including the base row and wall ends; not allowed with fade_height. All other vectors are LOCAL [width,depth,height]. Only use fields belonging to the chosen kind.",
     "parameters": obj({"expected_revision": {"type": "integer"}, "operations": {"type": "array", "items": OP, "minItems": 1, "maxItems": 20}}, ["expected_revision", "operations"])},
    {"name": "add_control_points", "description": "Refine the control grid without changing the surface shape: inserts per_span new control-point rows in every existing span along width (i) or height (j). Undoable. Indices change; read state again afterwards.",
     "parameters": obj({"expected_revision": {"type": "integer"}, "direction": {"type": "string", "enum": ["width", "height"]},
                        "per_span": {"type": "integer", "minimum": 1, "maximum": 4}}, ["expected_revision", "direction", "per_span"])},
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
