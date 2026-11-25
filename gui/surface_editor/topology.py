"""Plane creation, exact knot refinement and tolerance-limited grid rebuilding."""
import numpy as np
import rhino3dm as r


def counts(nu, nv):
    if any(type(n) is not int or not 4 <= n <= 20 for n in (nu, nv)):
        raise ValueError("U and V counts must be integers from 4 to 20 (cubic surfaces)")


def knots(n):
    return np.r_[np.zeros(4), np.arange(1, n-3)/(n-3), np.ones(4)]


def basis(n, ts):
    """Cubic B-spline basis for an open uniform knot vector."""
    t = knots(n)
    x = np.asarray(ts)[:, None]
    b = ((x >= t[:-1]) & (x < t[1:])).astype(float)
    for degree in range(1, 4):
        out = np.zeros((len(ts), len(t)-degree-1))
        for i in range(out.shape[1]):
            left = t[i+degree]-t[i]
            right = t[i+degree+1]-t[i+1]
            if left:
                out[:, i] += (x[:, 0]-t[i])/left*b[:, i]
            if right:
                out[:, i] += (t[i+degree+1]-x[:, 0])/right*b[:, i+1]
        b = out
    b[np.asarray(ts) == 1, -1] = 1
    return b


def from_points(points):
    nu, nv = points.shape[:2]
    counts(nu, nv)
    surface = r.NurbsSurface.Create(3, False, 4, 4, nu, nv)
    for collection, n in ((surface.KnotsU, nu), (surface.KnotsV, nv)):
        for k, value in enumerate(knots(n)[1:-1]):
            collection[k] = float(value)
    for i in range(nu):
        for j in range(nv):
            surface.Points[i, j] = r.Point4d(*map(float, points[i, j]), 1.)
    if not surface.IsValid:
        raise ValueError("Generated surface is invalid")
    return surface


def plane(width=3000., height=1500., nu=6, nv=6):
    counts(nu, nv)
    if any(isinstance(x, bool) or not isinstance(x, (int, float)) or not np.isfinite(x) or not 100 <= x <= 100000 for x in (width, height)):
        raise ValueError("Width and height must be between 100 and 100,000 mm")
    # Greville positions reproduce a linear parameterization exactly.
    u = np.array([knots(nu)[i+1:i+4].mean() for i in range(nu)])
    v = np.array([knots(nv)[i+1:i+4].mean() for i in range(nv)])
    x, z = np.meshgrid(width*u, height*v, indexing="ij")
    return from_points(np.stack([x, np.zeros_like(x), z], axis=-1))


def sample(surface, nu=65, nv=65):
    du, dv = surface.Domain(0), surface.Domain(1)
    return np.array([[[p.X, p.Y, p.Z] for v in np.linspace(dv.T0, dv.T1, nv)
                      for p in [surface.PointAt(u, v)]] for u in np.linspace(du.T0, du.T1, nu)])


def fit_axis(b, data):
    """Constrained least squares: keep both endpoint coefficients exact."""
    result = np.empty((b.shape[1], data.shape[1]))
    result[0], result[-1] = data[0], data[-1]
    residual = data - b[:, :1]*result[0] - b[:, -1:]*result[-1]
    result[1:-1] = np.linalg.lstsq(b[:, 1:-1], residual, rcond=None)[0]
    return result


def change_grid(surface, nu, nv, mode="refine", tolerance_mm=5.):
    counts(nu, nv)
    if mode not in ("refine", "rebuild"):
        raise ValueError("mode must be refine or rebuild")
    if isinstance(tolerance_mm, bool) or not isinstance(tolerance_mm, (int, float)) or not np.isfinite(tolerance_mm) or not 0 <= tolerance_mm <= 1000:
        raise ValueError("tolerance_mm must be from 0 to 1000")
    if mode == "refine":
        if nu < surface.Points.CountU or nv < surface.Points.CountV:
            raise ValueError("Refinement only adds points. Choose rebuild to reduce the count.")
        result = r.CommonObject.Decode(surface.Encode())
        for direction, target in ((0, nu), (1, nv)):
            while (result.Points.CountU if direction == 0 else result.Points.CountV) < target:
                collection = result.KnotsU if direction == 0 else result.KnotsV
                domain = result.Domain(direction)
                unique = np.unique([domain.T0, *[k for k in collection if domain.T0 <= k <= domain.T1], domain.T1])
                i = int(np.argmax(np.diff(unique)))
                if not collection.InsertKnot(float((unique[i]+unique[i+1])/2), 1):
                    raise ValueError("Knot insertion failed")
    else:
        su, sv = max(35, 3*nu+1), max(35, 3*nv+1)
        target = sample(surface, su, sv)
        first = fit_axis(basis(nu, np.linspace(0, 1, su)), target.reshape(su, -1)).reshape(nu, sv, 3)
        second = fit_axis(basis(nv, np.linspace(0, 1, sv)), first.transpose(1, 0, 2).reshape(sv, -1))
        result = from_points(second.reshape(nv, nu, 3).transpose(1, 0, 2))
    error = np.linalg.norm(sample(result)-sample(surface), axis=-1)
    maximum = float(error.max())
    if mode == "rebuild" and maximum > tolerance_mm + 1e-8:
        raise ValueError(f"Rebuild would deviate by {maximum:.3f} mm (sampled), above {tolerance_mm:g} mm. Use more points or a larger tolerance.")
    if not result.IsValid:
        raise ValueError("Resulting surface is invalid")
    return result, {"mode": mode, "grid_shape": [nu, nv], "max_sampled_deviation_mm": maximum,
                    "rms_sampled_deviation_mm": float(np.sqrt(np.mean(error**2))),
                    "note": "Refine preserves the shape. Rebuild is an approximation; deviations are sampled at matching normalized parameters, not a certified global bound."}
