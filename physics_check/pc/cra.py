"""Steps 5-6: compas_cra penalty solve and the per-interface tensile stress.

compas / compas_cra are imported inside ``_solve`` only, so
``polygon_section`` and ``interface_stress`` run without them.
"""

from __future__ import division

from .params import EPS, G, MM, TENSION_TOL


def polygon_section(pts2):
    """Area, centroid and central second moments (Ixx, Iyy, Ixy) of a polygon."""
    a = cx = cy = ixx = iyy = ixy = 0.0
    n = len(pts2)
    for i in range(n):
        x1, y1 = pts2[i]
        x2, y2 = pts2[(i + 1) % n]
        c = x1 * y2 - x2 * y1
        a += c
        cx += (x1 + x2) * c
        cy += (y1 + y2) * c
        ixx += (y1 * y1 + y1 * y2 + y2 * y2) * c     # integral y^2
        iyy += (x1 * x1 + x1 * x2 + x2 * x2) * c     # integral x^2
        ixy += (x1 * y2 + 2 * x1 * y1 + 2 * x2 * y2 + x2 * y1) * c
    a *= 0.5
    if abs(a) < EPS:
        return 0.0, (0.0, 0.0), 0.0, 0.0, 0.0
    cx /= 6.0 * a
    cy /= 6.0 * a
    ixx, iyy, ixy = ixx / 12.0, iyy / 12.0, ixy / 24.0
    if a < 0:
        a, ixx, iyy, ixy = -a, -ixx, -iyy, -ixy
    return a, (cx, cy), ixx - a * cy * cy, iyy - a * cx * cx, ixy - a * cx * cy


def interface_stress(points, origin, xaxis, yaxis, forces):
    """Net tensile stress from the CRA corner forces of one interface.

    Fits the linear stress distribution sigma = P/A + bending that reproduces
    the corner resultant P and its moments about the polygon centroid, and
    returns the largest tension at a polygon vertex: M/Z - P/A, using the
    contact polygon's own A and Z (biaxial form).  Inputs in m and the compas
    kg-force units; outputs in Pa / N / m.

    Returns dict(sigma_t, P, centroid (world), cop (world) or None, tensioned).
    """
    loc = []
    for q in points:
        d = [q[0] - origin[0], q[1] - origin[1], q[2] - origin[2]]
        loc.append((sum(d[i] * xaxis[i] for i in range(3)), sum(d[i] * yaxis[i] for i in range(3))))
    A, (cr, cs), Ixx, Iyy, Ixy = polygon_section(loc)
    Nf = [(f["c_np"] - f["c_nn"]) * G for f in forces]   # compression +
    tensioned = any(f["c_nn"] * G > TENSION_TOL for f in forces)
    P = sum(Nf)
    Mr = sum(n * (r - cr) for n, (r, _s) in zip(Nf, loc))   # first moment along x
    Ms = sum(n * (s - cs) for n, (_r, s) in zip(Nf, loc))   # first moment along y
    # sigma(x, y) = P/A + bx*x + by*y  with  [Iyy Ixy; Ixy Ixx] [bx; by] = [Mr; Ms]
    det = Iyy * Ixx - Ixy * Ixy
    if A <= 0 or abs(det) < 1e-30:
        sigma_t = float("inf") if tensioned else 0.0
    else:
        bx = (Mr * Ixx - Ms * Ixy) / det
        by = (Ms * Iyy - Mr * Ixy) / det
        sigma_t = max(-(P / A + bx * (r - cr) + by * (s - cs)) for r, s in loc)

    def world(r, s):
        return [origin[i] + r * xaxis[i] + s * yaxis[i] for i in range(3)]

    cop = world(cr + Mr / P, cs + Ms / P) if P > EPS else None
    return {"sigma_t": sigma_t, "P": P, "A": A, "centroid": world(cr, cs), "cop": cop,
            "tensioned": tensioned}


def _solve(frames, dims, supports, p):
    """Build the compas_cra assembly and run the penalty solve.

    Plain data in and out (lists, dicts), so it can run in a child process.
    frames: [[origin, xaxis, yaxis]] per node, in m; dims: [(W, L, H)] per
    node, in m (L differs for half bricks).
    Returns dict(state = 'solved' | 'geometry' | 'failed', n_interfaces,
    floating [node], edges [(u, v, [interface dict])], error).
    """
    import contextlib
    import io

    from compas.geometry import Box
    from compas.geometry import Frame
    from compas_assembly.datastructures import Block
    from compas_cra.algorithms import assembly_interfaces_numpy
    from compas_cra.datastructures import CRA_Assembly
    from compas_cra.equilibrium import cra_penalty_solve

    assembly = CRA_Assembly()
    for node, ((o, x, y), (W, L, H)) in enumerate(zip(frames, dims)):
        assembly.add_block(Block.from_shape(Box(W, L, H, frame=Frame(o, x, y))), node=node)
    assembly.set_boundary_conditions(supports)
    assembly_interfaces_numpy(assembly, nmax=p["nmax"], tmax=p["tmax"], amin=p["amin"])

    graph = assembly.graph
    n_itf = graph.number_of_edges()
    # isolated / floating blocks: free blocks not connected to a support
    seen = set(supports)
    stack = list(supports)
    while stack:
        n = stack.pop()
        for m in graph.neighbors(n):
            if m not in seen:
                seen.add(m)
                stack.append(m)
    floating = [n for n in range(len(frames)) if n not in seen]
    if floating:
        return {"state": "geometry", "floating": floating, "n_interfaces": n_itf}

    try:
        with contextlib.redirect_stdout(io.StringIO()):
            cra_penalty_solve(assembly, mu=p["mu"], density=p["density"], verbose=False, timer=False)
    except ValueError as err:
        return {"state": "failed", "error": str(err), "n_interfaces": n_itf}

    edges = []
    for u, v in graph.edges():
        itfs = []
        for itf in graph.edge_attribute((u, v), "interfaces"):
            if not itf.forces:
                continue
            fr = itf.frame
            itfs.append({"points": [list(q) for q in itf.points], "origin": list(fr.point),
                         "xaxis": list(fr.xaxis), "yaxis": list(fr.yaxis),
                         "forces": [dict(f) for f in itf.forces]})
        edges.append((u, v, itfs))
    return {"state": "solved", "n_interfaces": n_itf, "edges": edges}


def _solve_child(conn, args):
    """Child-process entry: run _solve and send the result back."""
    try:
        conn.send(("ok", _solve(*args)))
    except Exception as err:  # noqa: BLE001 - report anything to the parent
        conn.send(("error", "{}: {}".format(type(err).__name__, err)))
    finally:
        conn.close()


def _solve_with_timeout(args, timeout):
    """Run _solve in a spawned child process; kill it after timeout seconds.

    Returns the _solve dict, or dict(state='timeout') if the limit is hit.
    A thread or signal cannot stop the solver (it runs in compiled code, and
    Windows has no SIGALRM), so the solve runs in its own process.
    """
    import multiprocessing as mp

    ctx = mp.get_context("spawn")
    recv, send = ctx.Pipe(duplex=False)
    proc = ctx.Process(target=_solve_child, args=(send, args), daemon=True)
    proc.start()
    send.close()
    try:
        if not recv.poll(timeout):
            return {"state": "timeout", "n_interfaces": 0}
        try:
            kind, payload = recv.recv()
        except EOFError:
            return {"state": "failed", "error": "CRA process exited with code {}".format(
                proc.exitcode), "n_interfaces": 0}
    finally:
        recv.close()
        if proc.is_alive():
            proc.terminate()
        proc.join(5)
    if kind == "error":
        raise RuntimeError("CRA solve failed in the child process: " + payload)
    return payload


def run_cra(wall):
    """Step 5: penalty CRA solve, course 0 fixed.

    The solve is limited to p["cra_timeout"] seconds (0 = no limit, run in
    this process).  Returns dict(state = 'solved' | 'geometry' | 'failed' |
    'timeout', interfaces, ...).
    """
    p = wall.p
    nodes = list(wall.bricks())
    frames = []
    for b in nodes:
        o, x, y = wall.frame(b)
        frames.append([[c * MM for c in o], x, y])
    dims = [(wall.W * MM, b.length * MM, (wall.H + p["mortar_t"]) * MM) for b in nodes]
    supports = [n for n, b in enumerate(nodes) if b.k == 0]
    keys = ("nmax", "tmax", "amin", "mu", "density")
    args = (frames, dims, supports, {k: p[k] for k in keys})
    if p["cra_timeout"] > 0:
        res = _solve_with_timeout(args, p["cra_timeout"])
    else:
        res = _solve(*args)

    out = {"state": res["state"], "n_interfaces": res.get("n_interfaces", 0), "interfaces": []}
    if res["state"] == "geometry":
        out["floating"] = [nodes[n] for n in res["floating"]]
    if res["state"] == "failed":
        out["error"] = res.get("error")
    if res["state"] != "solved":
        return out

    allow = p["f_t"] / p["SF"]
    rows = []
    for u, v, itfs in res["edges"]:
        bu, bv = nodes[u], nodes[v]
        upper, lower = (bu, bv) if bu.k > bv.k else (bv, bu)
        for itf in itfs:
            st = interface_stress(itf["points"], itf["origin"], itf["xaxis"], itf["yaxis"],
                                  itf["forces"])
            st["upper"], st["lower"] = upper, lower
            st["ratio"] = st["sigma_t"] / allow if allow > 0 else float("inf")
            st["fails"] = st["tensioned"] and st["ratio"] > 1.0
            rows.append(st)
    out["interfaces"] = rows
    return out
