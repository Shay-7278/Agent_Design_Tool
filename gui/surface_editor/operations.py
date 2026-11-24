"""Provider-independent control-cage operations. No generated code is executed.

Coordinates are millimeters. Local axes are fixed at document load: width is
the horizontal U-edge direction, height is world Z, depth is height x width.
UV selections use normalized control-grid coordinates, not arc length.
"""
from __future__ import annotations

import numpy as np


def local_frame(cps):
    width = cps[-1].mean(axis=0) - cps[0].mean(axis=0)
    width[2] = 0
    if np.linalg.norm(width) < 1e-8:
        width = cps[:, -1].mean(axis=0) - cps[:, 0].mean(axis=0)
        width[2] = 0
    if np.linalg.norm(width) < 1e-8:
        width = np.array([1., 0., 0.])
    width /= np.linalg.norm(width)
    up = np.array([0., 0., 1.])
    return cps.mean(axis=(0, 1)), np.column_stack([width, np.cross(up, width), up])


def number(value, name, lo=-1e6, hi=1e6):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be numeric")
    value = float(value)
    if not np.isfinite(value) or not lo <= value <= hi:
        raise ValueError(f"{name} must be finite and between {lo} and {hi}")
    return value


def vector(value, size, name, lo=-1e6, hi=1e6):
    if not isinstance(value, list) or len(value) != size:
        raise ValueError(f"{name} requires {size} numbers")
    return np.array([number(x, name, lo, hi) for x in value])


FIELDS = {
    "translate": {"vector"}, "scale": {"factors"},
    "rotate": {"axis", "angle_degrees"},
    "twist": {"axis", "angle_degrees"},
    "bend": {"axis", "amount_mm"},
    "taper": {"axis", "factor"},
    "pull": {"center_uv", "radius_uv", "vector"},
    "smooth": {"strength", "iterations"},
    "flatten": {"axis", "strength"},
    "wave": {"axis", "amount_mm", "cycles", "phase_degrees", "fade_height"},
    "set_points": {"points"},
}


def apply_operations(cps, operations, origin, frame):
    """Compute a batch on a copy; the caller commits only if all operations pass.

rotate/scale pivot at current cage centroid. twist varies 0..angle from the
minimum to maximum along axis. bend adds a parabolic depth bulge (zero at
ends). taper scales transverse coordinates from 1 at the low end to factor
at the high end. flatten removes the chosen local coordinate about centroid.
All operations can preserve the outside rows/columns with keep_boundary.
"""
    if not isinstance(operations, list) or not 1 <= len(operations) <= 20:
        raise ValueError("operations must contain 1 to 20 operations")
    out = np.array(cps, dtype=float, copy=True)
    for op in operations:
        if not isinstance(op, dict) or op.get("kind") not in FIELDS:
            raise ValueError("unknown operation kind")
        kind = op["kind"]
        extra = set(op) - FIELDS[kind] - {"kind", "keep_boundary"}
        if extra:
            raise ValueError(f"unexpected fields for {kind}: {sorted(extra)}")
        keep = op.get("keep_boundary", False)
        if not isinstance(keep, bool):
            raise ValueError("keep_boundary must be boolean")
        before = out.copy()
        q = (out - origin) @ frame
        pivot = q.mean(axis=(0, 1))
        axis = op.get("axis", "height")
        if axis not in ("width", "depth", "height"):
            raise ValueError("axis must be width, depth or height")
        a = {"width": 0, "depth": 1, "height": 2}[axis]
        if kind == "translate":
            q += vector(op["vector"], 3, "vector")
        elif kind == "scale":
            q = pivot + (q - pivot) * vector(op["factors"], 3, "factors", .05, 20)
        elif kind in ("rotate", "twist"):
            angle = np.deg2rad(number(op["angle_degrees"], "angle_degrees", -180, 180))
            if kind == "twist":
                span = np.ptp(q[..., a])
                if span < 1e-8:
                    raise ValueError("cannot twist along a zero-length axis")
                angle = angle * (q[..., a] - q[..., a].min()) / span
            b, c = (a + 1) % 3, (a + 2) % 3
            x, y = q[..., b] - pivot[b], q[..., c] - pivot[c]
            q[..., b] = pivot[b] + x * np.cos(angle) - y * np.sin(angle)
            q[..., c] = pivot[c] + x * np.sin(angle) + y * np.cos(angle)
        elif kind in ("bend", "wave", "taper"):
            span = np.ptp(q[..., a])
            if span < 1e-8:
                raise ValueError("operation axis has zero length")
            t = (q[..., a] - q[..., a].min()) / span
            if kind == "taper":
                factor = number(op["factor"], "factor", .05, 10)
                for b in range(3):
                    if b != a:
                        q[..., b] = pivot[b] + (q[..., b] - pivot[b]) * (1 + t * (factor - 1))
            else:
                if a == 1:
                    raise ValueError("bend/wave axis must be width or height; displacement is depth")
                amount = number(op["amount_mm"], "amount_mm")
                if kind == "bend":
                    q[..., 1] += amount * 4 * t * (1 - t)
                else:
                    cycles = number(op.get("cycles", 1), "cycles", .01, 20)
                    # A coarse cage cannot resolve arbitrary frequencies.
                    count = out.shape[0] if a == 0 else out.shape[1]
                    if cycles > (count - 1) / 2:
                        raise ValueError(f"wave too detailed for {count} control points along {axis}; "
                                         "use fewer cycles or add_control_points first")
                    phase = np.deg2rad(number(op.get("phase_degrees", 0), "phase_degrees", -360, 360))
                    weight = 1.0
                    if "fade_height" in op:
                        if a != 0:
                            raise ValueError("fade_height applies only to width waves")
                        if keep:
                            raise ValueError("fade_height moves the base row, which keep_boundary would pin; "
                                             "drop keep_boundary (depth moves keep the base level)")
                        fade = number(op["fade_height"], "fade_height", .01, 1)
                        h = (q[..., 2] - q[..., 2].min()) / max(np.ptp(q[..., 2]), 1e-8)
                        weight = np.clip(1 - h / fade, 0, 1)
                    q[..., 1] += weight * amount * np.sin(2 * np.pi * cycles * t + phase)
        elif kind == "pull":
            center = vector(op["center_uv"], 2, "center_uv", 0, 1)
            radius = number(op["radius_uv"], "radius_uv", .01, 2)
            u, v = np.meshgrid(np.linspace(0, 1, out.shape[0]), np.linspace(0, 1, out.shape[1]), indexing="ij")
            falloff = np.exp(-((u-center[0])**2 + (v-center[1])**2) / (2*radius**2))
            q += falloff[..., None] * vector(op["vector"], 3, "vector")
        elif kind == "flatten":
            strength = number(op.get("strength", 1), "strength", 0, 1)
            q[..., a] += strength * (pivot[a] - q[..., a])
        elif kind == "smooth":
            strength = number(op.get("strength", .5), "strength", 0, 1)
            iterations = number(op.get("iterations", 1), "iterations", 1, 20)
            if int(iterations) != iterations:
                raise ValueError("iterations must be an integer")
            for _ in range(int(iterations)):
                average = (q[:-2, 1:-1] + q[2:, 1:-1] + q[1:-1, :-2] + q[1:-1, 2:]) / 4
                q[1:-1, 1:-1] += strength * (average - q[1:-1, 1:-1])
        elif kind == "set_points":
            points = op.get("points")
            if not isinstance(points, list) or not 1 <= len(points) <= out.shape[0]*out.shape[1]:
                raise ValueError("points must be a nonempty list no larger than the grid")
            seen = set()
            for p in points:
                if not isinstance(p, dict) or set(p) != {"i", "j", "position"}:
                    raise ValueError("each point requires i, j, position (world millimeters)")
                i, j = p["i"], p["j"]
                if type(i) is not int or type(j) is not int or not (0 <= i < out.shape[0] and 0 <= j < out.shape[1]):
                    raise ValueError("control-point index out of range")
                if (i, j) in seen:
                    raise ValueError("duplicate control-point index")
                seen.add((i, j))
                q[i, j] = (vector(p["position"], 3, "position") - origin) @ frame
        out = q @ frame.T + origin
        if keep:
            out[[0, -1], :] = before[[0, -1], :]
            out[:, [0, -1]] = before[:, [0, -1]]
        if not np.isfinite(out).all() or np.abs(out).max() > 1e6:
            raise ValueError("coordinates exceed the supported range of +/- 1,000,000 mm")
        if np.linalg.matrix_rank(out.reshape(-1, 3) - out.mean(axis=(0, 1)), tol=1e-5) < 2:
            raise ValueError("operation collapses the surface to a line or point")
    return out
