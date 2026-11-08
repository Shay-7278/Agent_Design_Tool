"""GhPython component: fits a brick wall to a surface, then checks and repairs it.

Paste this file into one GhPython component.  It does the Rhino part of the
workflow in ``physics_check_wf.md`` (steps 1-2: scale the surface in Z to a
whole number of courses, cut one section per course), writes the sections to
``physics_job.json``, runs the check loop (steps 2-7, the ``pc`` package)
through a subprocess (by default ``uv run --project <repo>
<repo>/physics_check/physics_check.py``, with the clone and ``uv`` found
automatically; see the ``repo`` and ``command`` inputs), reads
``physics_result.json`` back and
rebuilds the planes and boxes.  The result file also holds ``frames`` + brick
sizes in the ``export_frames_json.py`` format, so ``cra_check.py`` can open it
directly.

The module-level functions are pure Python; Rhino is imported only inside
``gh_main``, so ``course_layout`` etc. can be imported in plain CPython.

Axes follow the other scripts: frame Y = along the course (brickL), frame X =
overhang / wall thickness (brickW), Z up.  The wall run is world Y (the
direction the fit stretches).  Lengths in mm; f_t in Pa; density in kg/m3.

Component inputs (GhPython):
    surface:         Wall surface (Surface or Brep) [mm]
    brickL, brickW, brickH:  Brick dimensions [mm], default 190 / 90 / 57
    mortar_t:        Bed joint thickness [mm], default 10
    N:               Bricks per course, default 10
    head_joint:      Nominal head joint for the fit [mm], default 15
    min_head:        Minimum head joint [mm], default 10
    max_head:        Maximum head joint [mm], default 25
    offset_distance: Running-bond offset of odd courses along the course [mm],
                     default (brickL + head_joint) / 2
    face_offset:     0 if the surface is the wall centreline; otherwise the
                     signed offset [mm] from the surface to the centreline
                     (+/- brickW/2), positive towards the section's +X side
    density:         Brick density [kg/m3], default 700 (fired clay is
                     ~1800-2000; the tension demand scales with it)
    mu:              Friction coefficient, default 0.84
    f_t:             Mortar flexural tensile strength [Pa], default 0.2e6
    SF:              Safety factor on f_t, default 2.0
    max_overhang:    Max overhang of a brick past its support, perpendicular
                     to the wall [mm], default brickW / 2
    thickness_cap:   Max wall thickness [mm], default 3 * brickW
    min_contact:     Min bed overlap with the bricks below, as a fraction of
                     the bed area brickL*brickW, default 0.25
    max_iter:        Outer-loop iterations, default 10
    samples:         Points sampled per section curve, default 200
    job_path:        Folder for physics_job.json / physics_result.json;
                     default next to the saved .3dm
    repo:            Folder of the cloned repo (the one holding pyproject.toml
                     and physics_check/).  Not wired = searched upwards from
                     job_path and from the saved .3dm's folder, so it can be
                     left empty when the .3dm is saved inside the clone.
    command:         Command template run by the component, with {job} and
                     {result} replaced by the paths of the two files.  If the
                     command starts with wsl / wsl.exe they are WSL paths
                     (/mnt/c/...); otherwise native paths.
                     Not wired = ``default_command``: "<uv>" run --project
                     "<repo>" "<repo>/physics_check/physics_check.py" run
                     "{job}" "{result}", where <uv> is found on PATH or in
                     ~/.local/bin or ~/.cargo/bin.
                     Empty string = run in-process: compas_cra and the ``pc``
                     package (folder physics_check/) must then be importable
                     from Rhino's Python
    run:             Boolean - run the check when True
    (The solver tuning keys in ``pc/params.py`` DEFAULTS - l0_max_iter,
    move_step, max_move, min_step, slide_step, nmax, tmax, amin - are passed
    through too if an input with that name is added.)
Component outputs:
    Frames:          Brick centre planes after the checks
    Bricks:          Brick boxes (brickL x brickW x brickH)
    Status:          Per brick: passed / moved / flagged
    Deviation:       Per brick: plan distance from its fitted position [mm]
    InterfacePts:    Centroid of each CRA interface
    InterfaceRatio:  Per interface: sigma_t / (f_t / SF) (> 1 fails)
    FittedFrames:    Brick planes straight after the fit (step 2)
    Report:          Text summary (iterations, failures, flagged bricks)
"""

from __future__ import division
from __future__ import print_function

__author__ = "bn7278"
__version__ = "2026.11.06"

import json
import math
import os

# Inputs forwarded to the solver.  Keep in sync with pc/params.py DEFAULTS
# (the solver logs any name it does not know).
PARAM_NAMES = (
    "brickL", "brickW", "brickH", "mortar_t", "N", "head_joint", "min_head", "max_head",
    "offset_distance", "face_offset", "density", "mu", "f_t", "SF", "max_overhang",
    "thickness_cap", "min_contact", "max_iter", "l0_max_iter", "move_step", "max_move",
    "min_step", "slide_step", "nmax", "tmax", "amin",
)
BRICK_H = 57.0     # pc/params.py default brickH [mm]
MORTAR_T = 10.0    # pc/params.py default mortar_t [mm]

SOLVER_SCRIPT = os.path.join("physics_check", "physics_check.py")


def find_repo(start_dirs):
    """First folder holding physics_check/physics_check.py, searching each
    start dir and then its parents.  None if not found."""
    for d in start_dirs:
        if not d:
            continue
        d = os.path.abspath(d)
        while True:
            if os.path.isfile(os.path.join(d, SOLVER_SCRIPT)):
                return d
            parent = os.path.dirname(d)
            if parent == d:
                break
            d = parent
    return None


def find_uv(path_env=None, home=None):
    """Path of the uv executable: PATH first, then the default install folders
    (~/.local/bin, ~/.cargo/bin).  Rhino may hold a PATH from before uv was
    installed, hence the fallback.  None if not found."""
    exe = "uv.exe" if os.name == "nt" else "uv"
    if path_env is None:
        path_env = os.environ.get("PATH", "")
    home = home or os.path.expanduser("~")
    dirs = [d.strip('"') for d in path_env.split(os.pathsep) if d.strip()]
    dirs += [os.path.join(home, ".local", "bin"), os.path.join(home, ".cargo", "bin")]
    for d in dirs:
        f = os.path.join(d, exe)
        if os.path.isfile(f):
            return f
    return None


def default_command(repo, uv):
    """Command template that runs the solver with the repo's uv environment."""
    def esc(s):   # keep braces in paths out of str.format
        return s.replace("{", "{{").replace("}", "}}")
    script = os.path.join(repo, SOLVER_SCRIPT)
    return '"{}" run --project "{}" "{}" run "{{job}}" "{{result}}"'.format(
        esc(uv), esc(repo), esc(script))


def course_layout(h, brickH, mortar_t):
    """Z scaling to a whole number of courses: (n_courses, target_h, z_scale).

    Raises ValueError if the surface gives fewer than 2 courses.
    """
    pitch = brickH + mortar_t
    n = int(math.floor((h + mortar_t) / pitch + 1e-9))
    if n < 2:
        raise ValueError("surface too short: {:.1f} mm gives {} course(s)".format(h, n))
    target = n * pitch - mortar_t
    return n, target, target / h


def course_cut_heights(n_courses, brickH, mortar_t):
    """Mid-height of each course above the (scaled) surface base [mm]."""
    pitch = brickH + mortar_t
    return [k * pitch + 0.5 * brickH for k in range(n_courses)]


def win_to_wsl(path):
    """C:\\a\\b -> /mnt/c/a/b (unchanged if not a drive path)."""
    if len(path) > 1 and path[1] == ":":
        return "/mnt/" + path[0].lower() + path[2:].replace("\\", "/")
    return path.replace("\\", "/")


def uses_wsl(command):
    """True if the command template runs through WSL (starts with wsl / wsl.exe)."""
    words = command.split(None, 1)
    return bool(words) and os.path.basename(words[0].strip("\"'")).lower() in ("wsl", "wsl.exe")


def format_command(command, job_file, res_file):
    """Fill {job} / {result}: WSL paths for a wsl command, native paths otherwise."""
    if uses_wsl(command):
        job_file, res_file = win_to_wsl(job_file), win_to_wsl(res_file)
    return command.format(job=job_file, result=res_file)


def run_in_process(job):
    """Run the check inside Rhino's Python (command = "")."""
    from pc.loop import run_check
    return run_check(job)


def gh_main(g):
    import subprocess

    import Rhino
    import Rhino.Geometry as rg

    params = {k: g[k] for k in PARAM_NAMES if g.get(k) is not None}
    brickH = float(params.get("brickH", BRICK_H))
    mortar_t = float(params.get("mortar_t", MORTAR_T))
    out = {"Frames": [], "Bricks": [], "Status": [], "Deviation": [], "InterfacePts": [],
           "InterfaceRatio": [], "FittedFrames": [], "Report": "Set run to True"}

    srf = g.get("surface")
    if srf is None:
        out["Report"] = "No surface connected"
        return out
    brep = srf if isinstance(srf, rg.Brep) else (
        srf.ToBrep() if hasattr(srf, "ToBrep") else rg.Brep.CreateFromSurface(srf))
    if not g.get("run", False):
        return out

    tol = Rhino.RhinoDoc.ActiveDoc.ModelAbsoluteTolerance
    bb = brep.GetBoundingBox(True)
    z_base, h = bb.Min.Z, bb.Max.Z - bb.Min.Z
    try:
        n_courses, _target, z_scale = course_layout(h, brickH, mortar_t)
    except ValueError as err:
        out["Report"] = str(err)
        return out
    brep = brep.DuplicateBrep()
    base = rg.Plane(rg.Point3d(0, 0, z_base), rg.Vector3d.ZAxis)
    brep.Transform(rg.Transform.Scale(base, 1.0, 1.0, z_scale))

    samples = int(g.get("samples") or 200)
    sections = []
    for k, zc in enumerate(course_cut_heights(n_courses, brickH, mortar_t)):
        plane = rg.Plane(rg.Point3d(0, 0, z_base + zc), rg.Vector3d.ZAxis)
        crvs = rg.Brep.CreateContourCurves(brep, plane)
        crvs = rg.Curve.JoinCurves(crvs, 10 * tol) if crvs else []
        if not crvs:
            out["Report"] = "course {}: the surface has no section at z = {:.1f}".format(k, z_base + zc)
            return out
        crv = max(crvs, key=lambda c: c.GetLength())
        ts = crv.DivideByCount(samples, True)
        pts = [crv.PointAt(t) for t in ts]
        if not crv.IsClosed:
            pts.append(crv.PointAtEnd)
        sections.append({"points": [[q.X, q.Y] for q in pts], "closed": bool(crv.IsClosed)})

    job = {"z_base": z_base, "params": params, "sections": sections}
    doc_path = Rhino.RhinoDoc.ActiveDoc.Path
    doc_dir = os.path.dirname(doc_path) if doc_path else None
    folder = g.get("job_path") or doc_dir
    if not folder:
        out["Report"] = "No job_path given and the .3dm is not saved yet"
        return out

    command = g.get("command")
    if command is None:
        repo = g.get("repo") or find_repo([folder, doc_dir])
        if not repo or not os.path.isfile(os.path.join(repo, SOLVER_SCRIPT)):
            out["Report"] = ("Cannot find the cloned repo ({} missing). Wire its folder into "
                             "the repo input, or save the .3dm inside the clone.".format(SOLVER_SCRIPT))
            return out
        uv = find_uv()
        if not uv:
            out["Report"] = ("Cannot find uv on PATH or in ~/.local/bin / ~/.cargo/bin. Install uv "
                             "(https://docs.astral.sh/uv/), restart Rhino, or wire a command input.")
            return out
        command = default_command(repo, uv)

    job_file = os.path.join(folder, "physics_job.json")
    res_file = os.path.join(folder, "physics_result.json")
    with open(job_file, "w") as fh:
        json.dump(job, fh)

    if command == "":
        try:
            result = run_in_process(job)
        except ImportError as err:
            out["Report"] = ("in-process run needs the pc package (physics_check/) and compas_cra "
                             "on Rhino's Python path: {}".format(err))
            return out
    else:
        if os.path.exists(res_file):
            os.remove(res_file)
        cmd = format_command(command, job_file, res_file)
        try:
            subprocess.check_output(cmd, shell=True, stderr=subprocess.STDOUT)
        except subprocess.CalledProcessError as err:
            # exit code 1 = checks failed but a result was written
            if not os.path.exists(res_file):
                out["Report"] = "command failed:\n{}\n{}".format(cmd, err.output)
                return out
        with open(res_file) as fh:
            result = json.load(fh)

    def plane(f):
        o, x, y = f
        return rg.Plane(rg.Point3d(*o), rg.Vector3d(*x), rg.Vector3d(*y))

    hl, hw, hh = 0.5 * result["brickL"], 0.5 * result["brickW"], 0.5 * result["brickH"]
    for b in result.get("bricks", []):
        pl = plane(b["frame"])
        out["Frames"].append(pl)
        out["Bricks"].append(rg.Box(pl, rg.Interval(-hw, hw), rg.Interval(-hl, hl), rg.Interval(-hh, hh)))
        out["Status"].append(b["status"])
        out["Deviation"].append(b["deviation"])
        out["FittedFrames"].append(plane(b["fitted_frame"]))
    for i in result.get("interfaces", []):
        out["InterfacePts"].append(rg.Point3d(*i["centroid"]))
        out["InterfaceRatio"].append(i["ratio"])
    out["Report"] = result.get("report", "")
    return out


if __name__ == "__main__":
    try:
        import Rhino  # noqa: F401
        _IN_RHINO = True
    except ImportError:
        _IN_RHINO = False

    if _IN_RHINO:
        _o = gh_main(globals())
        Frames = _o["Frames"]
        Bricks = _o["Bricks"]
        Status = _o["Status"]
        Deviation = _o["Deviation"]
        InterfacePts = _o["InterfacePts"]
        InterfaceRatio = _o["InterfaceRatio"]
        FittedFrames = _o["FittedFrames"]
        Report = _o["Report"]
