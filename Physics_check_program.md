# Running the physics check

This explains how to run the brick wall physics check from a fresh clone of the `physics_check`
branch. The check takes a wall surface from Rhino, fits a running-bond brick wall to it, then
checks and repairs the wall:

- **L0 (geometry):** bed contact, head-joint collisions, overhang and thickness limits.
- **L1 (statics):** a compas_cra penalty solve with course 0 fixed, plus mortar tension
  `sigma_t = M/Z - P/A` against `f_t / SF`.

The full method is in `physics_check/physics_check_workflow.md`.

## How the pieces connect

```
Rhino surface ──► physics_check_gh.py (GhPython component, runs inside Rhino)
                    │  scales the surface in Z, cuts one section per course
                    │  writes physics_job.json
                    ▼
                  subprocess:  uv run physics_check/physics_check.py run job result
                    │  (plain CPython + compas_cra, the pc/ package)
                    │  fit -> L0 -> L1 -> repeat, writes physics_result.json
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
git clone -b physics_check https://github.com/Shay-7278/Agent_Design_Tool.git
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
| `physics_check/pc/` | Solver package: `fit`, `wall`, `l0`, `cra`, `l1`, `loop`, `params` |
| `physics_check/export_frames_json.py` | Optional component: export brick frames to JSON |
| `physics_check/physics_check_workflow.md` | The method, step by step |

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
json.dump({"z_base": 0.0, "params": {"N": 8, "density": 1900}, "sections": secs},
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
   | `N` | int | 10 | Bricks per course |
   | `head_joint` | float | 15 | Nominal head joint used in the fit [mm] |
   | `min_head`, `max_head` | float | 10 / 25 | Head joint range for repairs [mm] |
   | `offset_distance` | float | (brickL+head_joint)/2 | Running-bond offset of odd courses [mm] |
   | `face_offset` | float | 0 | 0 if the surface is the wall centreline, else ±brickW/2 |
   | `density` | float | 700 | Brick density [kg/m³]; fired clay is ~1800–2000 |
   | `mu` | float | 0.84 | Friction coefficient |
   | `f_t` | float | 0.2e6 | Mortar flexural tensile strength [Pa] |
   | `SF` | float | 2.0 | Safety factor on f_t |
   | `max_overhang` | float | brickW/2 | Max overhang past the support [mm] |
   | `thickness_cap` | float | 3·brickW | Max wall thickness [mm] |
   | `min_contact` | float | 0.25 | Min bed overlap as a fraction of brickL·brickW |
   | `max_iter` | int | 10 | Outer-loop iterations |
   | `samples` | int | 200 | Points sampled per section curve |
   | `job_path` | str | .3dm folder | Folder for `physics_job.json` / `physics_result.json` |

   Solver tuning keys (`l0_max_iter`, `move_step`, `max_move`, `min_step`, `slide_step`, `nmax`,
   `tmax`, `amin`; see `physics_check/pc/params.py`) are passed through if you add an input with
   that name.

5. Add the outputs: `Frames`, `Bricks`, `Status`, `Deviation`, `InterfacePts`,
   `InterfaceRatio`, `FittedFrames`, `Report`.

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
     minutes, depending on brick count and `max_iter`; Grasshopper freezes meanwhile),
   - reads `physics_result.json` back.
4. Connect a **Panel** to `Report`, and preview `Bricks`.

Set `run` back to False before you change inputs, or every change re-runs the solver.

## 6. Read the results

| Output | What it shows |
|---|---|
| `Report` | Fit stretch factors, one line per iteration (L0 pass/fail, CRA state, tensioned and failing interfaces), brick counts by status, max deviation, flagged bricks, failing interfaces, then `RESULT: PASS/FAIL` |
| `Bricks` / `Frames` | Bricks after the checks and repairs |
| `FittedFrames` | Bricks straight after the fit, before any repair (compare with `Frames`) |
| `Status` | Per brick: `passed`, `moved` or `flagged` (no position within the limits passed) |
| `Deviation` | Per brick: plan distance from its fitted position [mm] |
| `InterfacePts` / `InterfaceRatio` | Each CRA interface centroid and `sigma_t / (f_t/SF)`; above 1 fails |

Useful displays: colour `Bricks` by `Status` (List Item / Custom Preview), or colour
`InterfacePts` with a gradient on `InterfaceRatio`.

What the stop messages mean:

- **`surface too short`**: fewer than 2 courses fit the surface height.
- **Fit error for a course**: `N * (brickL + head_joint)` is shorter than the section's travel in
  X. Reduce the curvature or increase `N`.
- **Course cannot fit N bricks in [min_head, max_head]**: change `N` or widen the head joint range.
- **CRA state `geometry`**: some bricks float (not connected to course 0). It's a contact problem;
  check `min_contact` and the overhang.
- **CRA state `failed`**: the solve did not converge, most likely sliding. The loop reduces the
  overhang of those bricks; if it still fails, lower `max_overhang` or raise `mu`.
- **`max_iter reached`** / **`no brick could be moved further`**: the remaining failing bricks
  and interfaces are listed in the report.

`physics_result.json` holds everything above plus the full log. Its `frames`, `brickL/W/H` and
`mortar_t` keys use the same format as `export_frames_json.py`.

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
