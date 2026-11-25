"""Geometry core: load / edit / save a NURBS surface from a .3dm file.

Works without Rhino via `rhino3dm`. The control-point grid is exposed as an
(n, m, 3) numpy array; because a NURBS is linear in its control points,
moving CPs and re-evaluating gives the edited surface, and scaling all CPs
about a pivot scales the surface exactly (affine map on CPs == same map on
the surface).

rhino3dm quirks this works around (verified against rhino3dm 8.35):
- `NurbsSurfacePointList` has no named setter, but supports
  ``points[(i, j)] = rhino3dm.Point4d(x, y, z, w)``.
- ``BrepFace.ToNurbsSurface()`` returns a *detached copy*, so CP edits are
  made on that copy and the Brep is rebuilt on save via
  ``Brep.CreateFromSurface`` (degree, knots and domains are preserved).
- The edited object is replaced in the File3dm by GUID (kept on
  ``ObjectAttributes.Id``) so every other object in the file is untouched.

Result: the saved file stays a normal single-face Brep, compatible with the
existing Grasshopper / physics_check pipeline.
"""

from __future__ import annotations

import shutil
from pathlib import Path

import numpy as np
import rhino3dm


class NurbsModel:
    """A single-face NURBS surface backed by a rhino3dm Brep."""

    def __init__(
        self,
        path: Path,
        file: rhino3dm.File3dm,
        obj_id,
        attrs: rhino3dm.ObjectAttributes,
        srf: rhino3dm.NurbsSurface,
    ) -> None:
        self.path = Path(path)
        self._file = file
        self._obj_id = obj_id
        self._attrs = attrs
        self._srf = srf  # live, editable nurbs form (detached copy of the face)

    # ------------------------------------------------------------------ load
    @classmethod
    def flat(cls, width=3000., height=1500., nu=6, nv=6):
        from .topology import plane
        file = rhino3dm.File3dm()
        file.Settings.ModelUnitSystem = rhino3dm.UnitSystem.Millimeters
        return cls(Path.cwd() / "Untitled.3dm", file, None, rhino3dm.ObjectAttributes(),
                   plane(width, height, nu, nv))

    @classmethod
    def load(cls, path: str | Path) -> "NurbsModel":
        path = Path(path)
        if not path.exists():
            raise FileNotFoundError(path)
        file = rhino3dm.File3dm.Read(str(path))
        if file is None:
            raise IOError(f"could not read .3dm: {path}")
        for obj in file.Objects:
            g = obj.Geometry
            if isinstance(g, rhino3dm.Brep) and len(g.Faces) == 1:
                srf = g.Faces[0].ToNurbsSurface()
                return cls(path, file, obj.Attributes.Id, obj.Attributes, srf)
        raise ValueError(f"no single-face Brep surface found in {path}")

    # --------------------------------------------------------------- CP grid
    @property
    def shape(self) -> tuple[int, int]:
        """(n, m): control point count in U, V."""
        return self._srf.Points.CountU, self._srf.Points.CountV

    @property
    def degree(self) -> tuple[int, int]:
        return self._srf.Degree(0), self._srf.Degree(1)

    def control_points(self) -> np.ndarray:
        """CP grid as (n, m, 3); first axis is the U direction."""
        pts = self._srf.Points
        n, m = pts.CountU, pts.CountV
        out = np.empty((n, m, 3))
        for i in range(n):
            for j in range(m):
                p = pts.GetPoint(i, j)
                out[i, j] = (p.X, p.Y, p.Z)
        return out

    def weights(self) -> np.ndarray:
        """CP weights as (n, m). All 1.0 unless the surface is rational."""
        pts = self._srf.Points
        n, m = pts.CountU, pts.CountV
        out = np.empty((n, m))
        for i in range(n):
            for j in range(m):
                out[i, j] = pts.GetControlPoint(i, j).W
        return out

    def set_control_points(self, cps: np.ndarray) -> None:
        """Write an (n, m, 3) grid onto the live nurbs surface (keeps weights)."""
        n, m = self.shape
        if cps.shape != (n, m, 3):
            raise ValueError(f"expected CP grid {(n, m, 3)}, got {cps.shape}")
        w = self.weights()
        for i in range(n):
            for j in range(m):
                x, y, z = (float(v) for v in cps[i, j])
                self._srf.Points[(i, j)] = rhino3dm.Point4d(x, y, z, float(w[i, j]))

    def insert_knots(self, direction: int, per_span: int) -> None:
        """Add control points along U (0) or V (1) without changing the shape."""
        if direction not in (0, 1) or type(per_span) is not int or not 1 <= per_span <= 4:
            raise ValueError("direction must be 0 or 1 and per_span an integer from 1 to 4")
        knots =self._srf.KnotsU if direction == 0 else self._srf.KnotsV
        values = sorted(set(knots))
        for a, b in zip(values, values[1:]):
            for k in range(1, per_span + 1):
                if not knots.InsertKnot(a + (b - a) * k / (per_span + 1), 1):
                    raise RuntimeError("knot insertion failed")

    def snapshot(self) -> rhino3dm.NurbsSurface:
        return rhino3dm.CommonObject.Decode(self._srf.Encode())

    def restore(self, snapshot: rhino3dm.NurbsSurface) -> None:
        self._srf = rhino3dm.CommonObject.Decode(snapshot.Encode())

    # ------------------------------------------------------------- evaluate
    def evaluate(self, res_u: int = 60, res_v: int = 60) -> np.ndarray:
        """Tessellate the current surface as an (res_u, res_v, 3) point grid.

        Measured ~20 ms at 60x60 -- cheap enough to run every drag frame.
        """
        srf = self._srf
        du, dv = srf.Domain(0), srf.Domain(1)
        us = np.linspace(du.T0, du.T1, res_u)
        vs = np.linspace(dv.T0, dv.T1, res_v)
        out = np.empty((res_u, res_v, 3))
        for a, u in enumerate(us):
            for b, v in enumerate(vs):
                p = srf.PointAt(u, v)
                out[a, b] = (p.X, p.Y, p.Z)
        return out

    def centroid(self) -> np.ndarray:
        """Mean of the CP grid (a good pivot for scaling)."""
        return self.control_points().reshape(-1, 3).mean(axis=0)

    # ---------------------------------------------------------------- scale
    def scale(self, sx: float, sy: float, sz: float, pivot: np.ndarray | None = None) -> None:
        """Scale all CPs about `pivot` (default: CP centroid)."""
        if pivot is None:
            pivot = self.centroid()
        cps = self.control_points()
        cps = (cps - pivot) * np.array([sx, sy, sz]) + pivot
        self.set_control_points(cps)

    # ----------------------------------------------------------------- save
    def save(self, path: str | Path | None = None, backup: bool = True) -> Path:
        """Rebuild the Brep from the edited nurbs form and write the file.

        Default target is the loaded path. On the first overwrite a one-time
        backup ``<name>.3dm.orig`` of the untouched original is written.
        """
        out = Path(path) if path is not None else self.path
        if backup and self.path.exists() and out.resolve() == self.path.resolve():
            orig = self.path.with_suffix(".3dm.orig")
            if not orig.exists():
                shutil.copy2(self.path, orig)

        brep = rhino3dm.Brep.CreateFromSurface(self._srf)
        if brep is None or not brep.IsValid:
            raise RuntimeError("rebuilt Brep is invalid; refusing to save")
        if self._obj_id is not None:
            self._file.Objects.Delete(self._obj_id)
        self._obj_id = self._file.Objects.AddBrep(brep, self._attrs)
        ok = self._file.Write(str(out), version=7)
        if not ok:
            raise IOError(f"could not write .3dm: {out}")
        return out
