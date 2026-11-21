# Running the physics check

This explains how to run the brick wall physics check from a fresh clone of the `physics_check3`
branch. The check takes a wall surface from Rhino, fits a running-bond brick wall to it, then
checks and repairs the wall:

- **L0 (geometry):** bed contact, balance (`min_balance`), head-joint collisions, overhang
  limit. Bricks that can't be given enough contact anywhere are dropped (up to `max_drop`).
- **L1 (statics):** a compas_cra solve with course 0 fixed, plus mortar tension
  `sigma_t = M/Z - P/A` against `f_t / SF`. The `solver` input picks the solve:
  `cra_penalty` (default, the full CRA penalty formulation) or `rbe` (rigid block equilibrium,
  much faster, but it ignores block kinematics and can pass walls that `cra_penalty` fails;
  use it for quick iterations and `cra_penalty` for the final check).
- **Construction stages:** the partial wall (courses 0..K) is solved again after each course
  is laid, with no mortar tension (wet mortar has none). The report lists which bricks a robot
  arm has to hold at each stage and when each can be released. See
  [Construction stages](#construction-stages).

L1 only runs on a wall that passes L0. If bricks still fail L0 after the repairs, the check stops
there and reports them. The stage check only runs after the finished wall has been solved
successfully, and it doesn't change the PASS/FAIL result.

The surface sets the size of the wall: the number of courses comes from its height, and the
number of bricks in each course from that course's length and the course below. You choose the
brick size and the head joint range, not the counts. Course 0 gets its brick count from its
length. Each open course above it is laid in running bond over the head joints of the course
below: every bond brick is centred over a joint below, middle bricks are added or removed to
keep head joints near `head_joint`, and the ends are filled up to the surface edge (see
[End fill](#end-fill-end_overrun-end_support)).

The full method is in `physics_check/physics_check_workflow.md`.

## How the pieces connect

```
Rhino surface ──► physics_check_gh.py (GhPython component, runs inside Rhino)
                    │  scales the surface in Z, cuts one section per course
                    │  writes physics_job.json
                    ▼
                  subprocess:  uv run physics_check/physics_check.py run job result
                    │  (plain CPython + compas_cra, the pc/ package)
                    │  fit -> L0 -> L1 -> repeat -> construction stages,
                    │  writes physics_result.json
                    ▼
                  physics_check_gh.py reads the result back
                    └► Frames, Bricks, Status, Deviation, InterfacePts, InterfaceRatio, Report
```

Rhino's own Python can't load compas_cra, so the solver runs in a separate Python environment
managed by `uv`. The Grasshopper component calls it through a shell command, so the whole
check runs from one `run` toggle.

## 1. Requirements

- Windows with Rhino 8 and Grasshopper (Rhino 7 GhPython also works).
- Git.
- [uv](https://docs.astral.sh/uv/) (Python package and environment manager). In PowerShell:
  ```powershell
  powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"
  ```
  Open a new terminal afterwards so `uv` is on `PATH`. You don't need to install Python
  yourself: uv downloads Python 3.12 automatically (the version is pinned by `.python-version`).

## 2. Clone the branch

```powershell
cd C:\Users\<you>\Documents
git clone -b physics_check3 https://github.com/Shay-7278/Agent_Design_Tool.git
cd Agent_Design_Tool
```

In the rest of this file, `<REPO>` means the full path of this folder, e.g.
`C:\Users\<you>\Documents\Agent_Design_Tool`.

What matters in the clone:

| Path | Role |
|---|---|
| `pyproject.toml`, `uv.lock`, `.python-version` | Python environment (compas-cra 0.8.0, Python 3.12) |
| `physics_check/physics_check_gh.py` | The Grasshopper component (paste into GhPython) |
| `physics_check/physics_check.py` | Command-line solver called by the component |
| `physics_check/pc/` | Solver package: `fit`, `wall`, `l0`, `l0qp`, `cra`, `l1`, `stages`, `loop`, `params` |
| `physics_check/export_frames_json.py` | Optional component: export brick frames to JSON |
| `physics_check/physics_check_workflow.md` | The method, step by step |
| `gui/surface_editor/` | Surface editor GUI: edit the surface and run the check without Rhino (see [section 7](#7-run-without-rhino-the-surface-editor-gui)) |

## 3. Set up the Python environment

From `<REPO>`:

```powershell
uv sync
```

This creates `<REPO>\.venv` and installs compas, compas_cra, numpy, scipy and shapely from the
lock file. Check that it works:

```powershell
uv run python -c "import compas_cra; print('compas_cra OK')"
uv run physics_check\physics_check.py --help
```

### Optional: test the solver without Rhino

Save this as `make_job.py` in `<REPO>`. It writes an 8-course test wall: a sine section that
leans 8 mm per course.

```python
import json, math, sys
secs = []
for k in range(8):
    lean = 8.0 * k  # mm per course in X
    pts = [[lean + 40 * math.sin(2 * math.pi * t / 200.0), 2200.0 * t / 200.0] for t in range(201)]
    secs.append({"points": pts, "closed": False})
json.dump({"z_base": 0.0, "params": {"density": 1900}, "sections": secs},
          open(sys.argv[1], "w"))
```

Then run it:

```powershell
uv run make_job.py job.json
uv run physics_check\physics_check.py run job.json result.json            # L0 + L1
uv run physics_check\physics_check.py run job.json result.json --no-cra   # L0 only
```

The report is printed to the terminal and ends with `RESULT: PASS` or `RESULT: FAIL`. The exit
code is 0 on pass and 1 on fail; a fail still writes `result.json`.

## 4. Set up the Grasshopper component

1. Open Rhino, model or open your wall surface, and **save the .3dm**. By default the job and
   result files are written next to the saved .3dm.
2. Open Grasshopper and add a Python script component: **Python 3 Script** in Rhino 8, or
   **GhPython** in Rhino 7.
3. Open the editor and paste the whole content of `physics_check/physics_check_gh.py`.
4. Add the inputs below (zoom in on the component and use the ⊕ buttons). Names must match
   exactly. Set the type hints shown. Inputs left unwired use the defaults.

   | Input | Type hint | Default | Meaning |
   |---|---|---|---|
   | `surface` | Brep | – | Wall surface [mm] |
   | `run` | bool | False | Runs the check when True |
   | `repo` | str | auto | Clone folder `<REPO>`; needed only if the .3dm is saved outside the clone |
   | `command` | str | auto | Shell command that runs the solver; leave unwired (see below) |
   | `brickL`, `brickW`, `brickH` | float | 190 / 90 / 57 | Brick size [mm]; L along the wall, W = overhang direction |
   | `mortar_t` | float | 10 | Bed joint [mm] |
   | `head_joint` | float | 15 | Nominal head joint g [mm]. Course 0 follows `baseline.gh`: a full brick flush with each end and a middle-brick count from its length. Courses above it keep head joints near g by adding or removing middle bricks, and use g for the end fill (see `physics_check_workflow.md` step 2) |
   | `min_head`, `max_head` | float | 5 / 25 | Head joint range used when the courses are laid out, on all courses [mm]. `min_head` is also the L0 collision check; `max_head` is not an L0 check. A course 0 or closed course that has to drop a brick on a tight curve may end up with joints over `max_head` (reported) |
   | `end_tol` | float | 50 | Bond layout: a course end further than this from the surface edge [mm] is reported |
   | `offset_distance` | float | (brickL+head_joint)/2 | Running-bond offset of odd closed courses [mm] |
   | `face_offset` | float | 0 | 0 if the surface is the wall centreline, else ±brickW/2 |
   | `density` | float | 700 | Brick density [kg/m³]; fired clay is ~1800–2000 |
   | `mu` | float | 0.84 | Friction coefficient |
   | `f_t` | float | 0.2e6 | Mortar flexural tensile strength [Pa] |
   | `SF` | float | 2.0 | Safety factor on f_t |
   | `max_overhang` | float | brickW/2 | Max overhang past the support [mm] |
   | `min_contact` | float | 0.25 | Min total bed overlap as a fraction of the brick's own bed area (half bricks: brickL/2·brickW) |
   | `min_balance` | float | 10 | Min distance of a brick's centre inside its contact region with the course below, along the course and across the wall [mm]. A brick resting on the end of a single brick fails; one bridging a joint passes |
   | `use_cra` | bool | True | Run L1 (and the stage check); False = fit + L0 only |
   | `solver` | str | `cra_penalty` | L1 solver: `cra_penalty` or `rbe` (see the top of this file) |
   | `end_overrun` | float | 1 | Bond layout: an end-fill piece (full brick or half bat) may run past the surface edge by up to this [mm]; 0 = never past the edge |
   | `end_support` | bool | True | Bond layout: an end-fill piece is placed only if it bears on the course below by itself (`min_contact`, `min_balance`); otherwise the fill is cut back from the edge. False = the old fill |
   | `max_iter` | int | 10 | Outer-loop iterations |
   | `cra_timeout` | float | 300 | Time limit per CRA solve [s]; 0 = no limit |
   | `samples` | int | 200 | Points sampled per section curve |
   | `job_path` | str | .3dm folder | Folder for `physics_job.json` / `physics_result.json` |

   Solver tuning keys are passed through if you add an input with that name (see
   `physics_check/pc/params.py`):
   - `l0_method`: L0 repair, `qp` (default: all bricks moved together to the closest passing
     layout, with plan rotation) or `greedy` (one brick at a time);
   - `qp_trust`, `qp_max_iter`, `max_rotation` (largest brick rotation from the course
     direction, default 30°; 0 = none), `qp_smooth` (spreads a deviation along the course);
   - `l0_max_iter`, `move_step`, `max_move` (largest deviation from the fit, default 0 = no
     limit), `min_step`, `slide_step`, `nmax`, `tmax`, `amin`.

   A few settings are not component inputs yet, so their defaults always apply from
   Grasshopper: `max_drop` (most bricks the QP repair may drop per call, default 10; 0 = never
   drop) and the construction-stage settings (`stage_check`, `stage_solver`, `stage_f_t`,
   `stage_max_tries`). To change them, add them to `"params"` in `physics_job.json` and run the solver
   from the command line (see [Troubleshooting](#troubleshooting)).

5. Add the outputs: `Frames`, `Bricks`, `Status`, `Deviation`, `InterfacePts`,
   `InterfaceRatio`, `FittedFrames`, `Report`, `SolveTime`.

### How the solver command is found

With `command` unwired, the component builds the command itself:

```
"<uv>" run --project "<REPO>" "<REPO>\physics_check\physics_check.py" run "{job}" "{result}"
```

- **`<REPO>`** is the `repo` input if wired. Otherwise the component searches upwards from
  `job_path` and from the .3dm's folder for a folder containing
  `physics_check\physics_check.py`. If you save your .3dm anywhere inside the clone, you don't
  need to wire anything. If it lives elsewhere, wire a **Panel** with the clone path into `repo`.
- **`<uv>`** is found on `PATH`, or else in the default install folders
  `%USERPROFILE%\.local\bin` and `%USERPROFILE%\.cargo\bin`. That means it still works if Rhino
  was opened before uv was installed.

The component replaces `{job}` and `{result}` with the two file paths. `--project` makes uv use
the clone's environment no matter what folder Rhino runs in.

**Custom command (optional).** Wire a string into `command` to override the default, e.g. for
WSL (the clone and uv live inside WSL). Start the command with `wsl` / `wsl.exe`, and the
component converts the file paths to `/mnt/c/...` automatically:

```
wsl.exe -e bash -lc "cd /home/<you>/Agent_Design_Tool && ~/.local/bin/uv run physics_check/physics_check.py run '{job}' '{result}'"
```

The .3dm (or `job_path`) must then sit on a Windows drive so WSL can reach it through `/mnt/c`.

An empty string runs the solver inside Rhino's Python. That only works if compas_cra and the
`physics_check/` folder are importable there, so don't use it unless you've set that up.

## 5. Run the check

1. Wire the surface into `surface` and a **Boolean Toggle** into `run` (plus `repo` if the .3dm is
   saved outside the clone).
2. Set `density` to your brick's real value. The tension demand scales directly with it.
3. Set `run` to True. The component:
   - scales the surface in Z to a whole number of courses (it stops if fewer than 2 fit),
   - cuts one section per course at mid-height,
   - writes `physics_job.json`, runs the solver and waits for it (this can take from seconds to
     minutes, depending on brick count and `max_iter`; Grasshopper freezes meanwhile, and each
     CRA solve is capped at `cra_timeout`),
   - reads `physics_result.json` back.
4. Connect a **Panel** to `Report`, and preview `Bricks`.

Set `run` back to False before you change inputs, or every change re-runs the solver.

## 6. Read the results

| Output | What it shows |
|---|---|
| `Report` | Bricks per course (and any course that lost a brick on a tight curve), the head joint range, bond end gaps/overhangs and any end fill cut back by `end_support`, one line per iteration (L0 pass/fail, CRA state, tensioned and failing interfaces), the construction stages, brick counts by status, max deviation, flagged bricks, failing interfaces, then `RESULT: PASS/FAIL` |
| `Bricks` / `Frames` | Bricks after the checks and repairs |
| `FittedFrames` | Bricks straight after the fit, before any repair (compare with `Frames`) |
| `Status` | Per brick: `passed`, `moved`, `flagged` (no position within the limits passed) or `failed` (still failing L0 or L1 when the loop stopped) |
| `Deviation` | Per brick: plan distance from its fitted position [mm] |
| `InterfacePts` / `InterfaceRatio` | Each CRA interface centroid and `sigma_t / (f_t/SF)`; above 1 fails. With `f_t = 0` the ratio has no finite value, so a tensioned interface shows `sigma_t` in kPa instead (any value above 0 fails) |
| `SolveTime` | Total run time seen by the component (including uv and Python start-up), the check loop's own time, and the time spent in CRA solves [s] |

Dropped bricks are not in `Bricks` / `Frames`. The report lists them with the reason
(`course k brick i: dropped (...)`), and `physics_result.json` has them under `"dropped"`.

Useful displays: colour `Bricks` by `Status` (List Item / Custom Preview), or colour
`InterfacePts` with a gradient on `InterfaceRatio`.

What the stop messages mean:

- **`surface too short`**: fewer than 2 courses fit the surface height.
- **Course too short**: an open section can't hold its end bricks plus one middle brick, or a
  closed one can't hold three bricks. Check the surface near that height.
- **Course cannot keep every head joint >= min_head**: the course is too tightly curved for its
  bricks. Widen `[min_head, max_head]`, use a shorter `brickL`, or reduce the curvature.
- **CRA state `geometry`**: some bricks float (not connected to course 0). It's a contact problem;
  check `min_contact` and the overhang.
- **CRA state `failed`**: the solve did not converge, most likely sliding. The loop reduces the
  overhang of those bricks; if it still fails, lower `max_overhang` or raise `mu`.
- **`L0 fail (... brick(s) still failing); CRA skipped`**: L0 couldn't repair every brick, so L1
  didn't run. The flagged bricks and their failing checks are listed below the summary; fix those
  (surface, `max_overhang`, `min_contact`, head joint range) and run again.
- **`CRA timed out after ... s; L1 not checked`**: one solve took longer than `cra_timeout`, so
  the loop stopped and the statics were not checked (the result is `FAIL`). Large walls take
  longer to solve; raise `cra_timeout` (0 = no limit).
- **`max_iter reached`** / **`no brick could be moved further`**: the remaining failing bricks
  and interfaces are listed in the report.

`physics_result.json` holds everything above plus the full log. Its `frames`, `brickL/W/H` and
`mortar_t` keys use the same format as `export_frames_json.py`. The stage results are under
`"stages"` (see below).

### Construction stages

L1 checks the finished wall. A finished wall that passes can still be unstable while it's being
built:

- a brick can depend on the course above it, which isn't there yet;
- a lean that builds up over several courses can put a partial stack's centre of mass outside
  its supports (L0 only compares each brick with the course directly below);
- L1 may rely on mortar tension (`f_t > 0`), which wet mortar doesn't have.

So for every K = 1..top course, the check solves courses 0..K with no tension and reports:

- **bricks to hold**: the bricks above any contact that would need tension;
- **held_ok**: whether holding those bricks makes the stage stable;
- **one arm**: when several bricks need holding, whether holding just one of them is enough
  (up to `stage_max_tries` bricks are tried, top courses first);
- **hold spans**: each held brick is held from the first stage that needs it until the first
  later stage that's stable without it, usually once the next course is laid. A brick that
  still needs holding when the wall is finished is reported as unstable as built.

Example report lines (bricks are `course/index`):

```
construction stages (lp, courses 0..K, f_t = 0 kPa; bricks as course/index): 22 stage(s), 2 need holding, 0 need more than one held brick, 1.36 s
  stage 10: hold 1 brick(s) 10/9; one arm: hold 10/9
  brick 10/9: hold from stage 10 through stage 10 (released once course 11 is laid)
```

Settings (in `pc/params.py`; not Grasshopper inputs yet):

| Setting | Default | Meaning |
|---|---|---|
| `stage_check` | True | Run the stage check after L1 |
| `stage_solver` | `"lp"` | `"lp"`: a linear program on the L0 contact polygons that minimises total corner tension, solved with HiGHS (~0.05 s per stage). Checks `f_t = 0` only and assumes horizontal contacts (bricks rotated in plan only). `"rbe"`: compas_cra with the main `solver`, uses `stage_f_t` (a few seconds per stage) |
| `stage_f_t` | 0 | Tensile strength during construction [Pa]; `rbe` only |
| `stage_max_tries` | 6 | Most single bricks tried as the one held brick, per stage |

In `physics_result.json`, `"stages"` is
`{f_t, stages: [{course, state, need, held_ok, one_arm, arms, time}], holds: [{brick, from, to}], max_arms, unknown}`,
with bricks as `[course, index]`. Only whole courses are checked, not the states while a course
is half laid; those depend on the robot's placement order.

### End fill (`end_overrun`, `end_support`)

In the bond layout, each course is laid over the head joints of the course below, and the gap
left at each end of the course up to the surface edge is filled with a full brick or a half bat.
Two rules control that fill:

- **`end_overrun`**: a fill piece counts as fitting if it runs past the surface edge by no more
  than this amount. Before, a gap just 0.5 mm too small for a full brick got a half bat and an
  open gap of ~85 mm. The overrun shows as a negative value in the report's
  "bond end gaps/overhangs" line.
- **`end_support`**: a fill piece is placed only if it bears on the course below by itself,
  using the same limits as L0 (`min_contact`, `min_balance`). If it doesn't, a full brick is
  tried as a half bat, then pieces are left out from the outside in. Where the course below
  stops short of the edge, the course steps back (racks) instead of bridging the gap with an
  unsupported end brick. Cut-back courses are listed in the report as
  `course k start/end` (pieces left out; 0.5 = full brick swapped for a half bat).

The trade-off is that a course end can now stop further from the surface edge. Ends that need
holding during construction are usually unsupported fill pieces, and these two rules remove
most of them. On the optionA test wall (rbe, `f_t = 0`), no stage needs a held brick (before,
brick 13/0 had to be held at stage 13), and the largest move from the fitted position dropped
from 31.5 mm to 2.1 mm. Set `end_overrun = 0` and `end_support = False` to get the previous
layout.

## 7. Run without Rhino: the surface editor GUI

`gui/surface_editor` edits the wall surface and runs the same check without Rhino or
Grasshopper. It opens the .3dm with `rhino3dm` and shows it in a PyVista window. You reshape
the surface by dragging its control points. One key cuts the course sections in Python (the same
steps as `physics_check_gh.py`), runs `physics_check.py` and draws the fitted brick wall,
coloured by status.

`uv sync` (section 3) already installs `pyvista` and `rhino3dm`; nothing else is needed.

### Start it

From the `gui` folder:

```powershell
cd <REPO>\gui
uv run --project .. python -m surface_editor
```

With no arguments it opens `<REPO>\Reference Surface_1.3dm`; if that file is missing, it opens the
first `.3dm` found in `<REPO>`. To open a different surface, or show the wall from a particular
result:

```powershell
uv run --project .. python -m surface_editor "C:\path\to\surface.3dm"
uv run --project .. python -m surface_editor "C:\path\to\surface.3dm" -r "C:\path\to\result.json"
```

The surface must be a single-face Brep. Close Rhino first if it has the file open, or saving
(`s`) can fail because Rhino locks the file.

### Edit the surface

- **Drag an orange sphere** to move that control point. Only the nearby part of the surface
  changes.
- **Sliders** (right side): `scale X`, `scale Y`, `scale Z` scale the whole surface about its
  centre (0.5–1.5); `soft pull` makes a drag also pull the neighbouring control points, falling
  off with distance (0 = move only the dragged point).

| Key | Action |
|---|---|
| `x` / `y` / `z` | Lock drags to that axis |
| `g` | Free drag (no axis lock) |
| `u` | Undo (up to 64 steps) |
| `b` | Reset all edits back to the loaded surface |
| `a` | Run the check on the current surface (see below) |
| `v` | Show / hide the brick wall |
| `s` | Save: overwrites the .3dm (a one-time backup `<name>.3dm.orig` is written first) |
| `r` | Reset the camera |
| `q` | Quit |

You don't need to save before pressing `a`; it uses the surface as it is on screen. Save only to
keep the edit.

### Run the check (`a`)

1. Press `a`. The status line shows progress. Cutting the sections takes under a second; the
   solve takes about 20–90 s and the window stays usable meanwhile.
2. When it finishes, the wall is redrawn on the new surface, coloured by status (`passed`,
   `moved`, `flagged`, `failed`; tensioned interfaces are marked in red), and the status line
   shows the brick count and `PASS` / `FAIL`.

Files are written to `<REPO>\results\`: `gui_refit_job.json` and `gui_refit_result.json`
(overwritten on each run). To read the full report, run the job by hand as in
[Troubleshooting](#troubleshooting).

**Parameters.** The GUI has no inputs for brick size, density and so on. `a` copies them from the
result of the wall currently shown (or the newest `*_result.json` in `results\`). With no earlier
result, it uses the solver defaults from `physics_check/pc/params.py`. To run with other
parameters, write a result with them first (for example by running a job with your `"params"`
from the command line, section 3), then open the GUI with `-r` pointing at that result. `a`
always runs L1 (CRA); the L1 solver follows the `solver` parameter (`rbe` is much faster for
iterating).

**Limits compared with Grasshopper.** Sections are cut by sampling the surface, so a course end
can be off by up to about 11 mm from Rhino's exact contour on heavily trimmed courses. Use the
Grasshopper component for the final check.

## Troubleshooting

| Symptom | Fix |
|---|---|
| `Report` says `Set run to True` | Toggle `run` |
| `No job_path given and the .3dm is not saved yet` | Save the .3dm or wire a folder into `job_path` |
| `Cannot find the cloned repo` | Wire the clone root (the folder with `pyproject.toml`) into `repo`, or save the .3dm inside the clone |
| `Cannot find uv` | Install uv, then restart Rhino; or wire a `command` with the full path to `uv.exe` |
| `command failed: ... can't open file` | `repo` points at the wrong folder; it must be the clone root |
| `ModuleNotFoundError: compas_cra` | Run `uv sync` in `<REPO>`; with a custom `command`, make sure it uses `--project <REPO>` or `cd /d <REPO>` |
| First run is slow | uv is still installing the environment; later runs are faster |
| `course k: the surface has no section at z = ...` | The surface has gaps or doesn't span the full height at that course |

To debug outside Grasshopper, run the job file the component wrote by hand:

```powershell
cd <REPO>
uv run physics_check\physics_check.py run "C:\path\to\physics_job.json" result.json
```
