# Chat-enabled surface editor

This adds a Qt chat interface to the existing PyVista/rhino3dm application.
Rhino is not required. The original `python -m surface_editor` interface still works;
add `--ai` for the new window. Source surfaces are saved only through **Save surface as**.

## Start

From the repository in PowerShell:

```powershell
uv sync --extra ai
.\launch_ai.ps1
```

Or run directly (also works if PowerShell blocks scripts):

```powershell
cd gui
..\.venv\Scripts\python.exe -m surface_editor --ai
```

Optional: append a surface path and/or `-r path_to_result.json`.
Without a surface path, the editor starts with a new **3000 × 1500 mm vertical
flat wall**, with a **6 × 6 cubic control grid**, and no old brick overlay.
To open the original reference, pass `"..\Reference Surface_1.3dm"` from `gui/`.

## Navigation, undo and changing the grid

- **Right-drag:** orbit around world Z while keeping the view upright. Horizontal
  movement changes azimuth; vertical movement changes elevation, clamped short of
  the poles so the view cannot turn upside down. This changes the camera, not geometry.
- **Shift + right-drag:** pan. **Mouse wheel:** zoom. **Reset view:** restore the
  upright isometric view. Left-drag on empty space also uses upright orbit;
  middle-drag or Shift + left-drag pans.
- **Left-drag a handle:** move a control point. X/Y/Z still constrain point movement.
- **Ctrl+Z:** undo a surface edit when the viewport has focus. **Ctrl+Y** or
  **Ctrl+Shift+Z:** redo. Toolbar buttons work regardless of focus. Text fields
  keep their usual text undo, so editing a prompt does not undo geometry.
- One mouse drag is one undo step, even if you pause during the drag. Undo includes
  full geometry, knots, weights, grid dimensions and scale-slider state.
- **New flat wall…:** set width, height and U/V point counts; replacement is undoable.
- **Rebuild grid…:** set 4–20 points per direction. **Refine** adds knots and preserves
  the surface exactly. **Rebuild** fits a uniform cubic grid, allowing fewer points;
  it reports sampled deviation and refuses to exceed the tolerance you entered.
  The deviation is measured on a 65 × 65 grid at corresponding normalized parameters,
  not a guaranteed global/Hausdorff error bound. Rebuilding preserves the four corners;
  the rest of the boundary is fitted and may change within the sampled tolerance.

The original viewport used free trackball camera rotation, which could roll. The new
right-drag navigation fixes the up direction to world Z. It provides familiar CAD
orbit behavior without requiring Rhino.

## Window openings

Use **Window…** on the toolbar to insert/edit the first rectangular opening.
Choose its position along the wall (0–1), width, height and sill elevation.
The initial dialog values fit the default wall. Your example of a 900 × 1200 mm
window with an 800 mm sill needs a taller wall: create a 2500 mm-high wall first.

After insertion, four **cyan corner handles** resize the rectangle. They are
displayed slightly in front of the wall so bricks cannot hide them; dragging is
projected back onto the surface. Use numeric fields to move the opening precisely.
The NURBS control points remain unchanged. Ctrl+Z/Ctrl+Y includes opening edits,
and **Remove opening** is available in the same dialog.

AI examples:
- “Add a centered 900 × 600 mm window with its sill 500 mm above the ground.”
- “Make the existing window 1000 mm wide.”
- “Move the window to 40 percent of the wall length.”
- “Remove the window.”

The AI uses `set_opening` and `remove_opening`. The document stores a list of openings,
but this first version accepts **one opening**. Openings save inside the `.3dm`
document under `surface_studio.openings.v1`; reopen that file to restore them.
Other Rhino readers still see the complete untrimmed surface.

Width is arc length along each horizontal course, centered at the chosen fraction
of that course. Sill is above the wall base. Sill rounds **down**, head rounds **up**
to whole courses; the UI reports the effective dimensions explicitly. For example,
with a 57 mm brick and 10 mm bed joint, a requested 500–1100 mm opening becomes
469–1139 mm. This avoids cutting bricks vertically. The preview accounts for the
existing generator's vertical course-layout scaling.

The opening-aware generator splits valid placement intervals **before** creating
bricks, fits each interval with a shared course-parity preference, and prefers full
and half bricks. Optional custom-length pieces occur only at interval ends and
respect the specified minimum length. Results include `custom_piece_schedule` with
course/index and cut lengths. Passing custom pieces appear teal. Concealed lintels
are hidden; failed masonry checks retain the failure colors.

The opening preview is a translucent extrusion through the surface, fixed in the
document depth direction. Orbiting changes only the camera; the volume and its
handles stay fixed in world coordinates. The NURBS remains complete.
Press **A with the viewport focused**, or use **Generate + analyze (A)**, to generate
and run full analysis. A failed generation keeps any previous wall visible and
explicitly marks the run as failed.

On curved courses, boundary pieces may be shortened further to keep their complete
footprints inside their placement intervals. Custom pieces respect the minimum
length; results expose these trims in `boundary_adjustments`. The trims can widen
adjacent joints; this version does not redistribute the course after trimming.

**First-version analysis limits:**
- Opening layouts run an iterative **L0 QP repair → L1 equilibrium → L1-guided
  movement → recheck** search. The existing trust-region QP uses exact linearized
  opening/contact constraints. Accepted moves cannot cross opening boundaries;
  the lintel stays in place. Fast mode performs L0 repair only. Full mode runs L1
  once L0 is valid; exhausted L0 repair is reported explicitly.
- Repair retains each interval's piece count and custom lengths. Automatic brick
  dropping and rebonding across regions are disabled. The bounded search may fail;
  it does not guarantee a solution or search all possible bond arrangements.
  `optimization_history` records L0/L1 states, feedback moves and iteration count.
- An enabled lintel is a **hidden, massless, zero-thickness rigid support** beneath
  the head-course masonry. It never replaces a brick course. Its bearing contacts
  transfer load into both jambs; it is not grounded. Direct masonry contacts are
  clipped around the plate footprint to avoid counting bearing area twice.
- Concealed-lintel equilibrium uses the selected **RBE or CRA penalty solver**,
  with explicit clipped bearing polygons and the configured friction and tensile
  limits. The configured construction-stage solver is retained (default: no-tension
  LP for wet mortar). Lintel self-weight, capacity, section design and deflection are
  **not checked**. A pass is conditional on the ideal support assumption.
  Curved lintels currently use a straight chord proxy and must pass contact checks.
  In CRA penalty mode, the ideal plate's top contact has zero closure, so inserting
  the plate does not double the mortar-contact closure along that load path.
- Construction stages are checked at complete-course boundaries and assume the
  lintel is installed before masonry above. Installation within a course is not modeled.
- Single-branch open walls are supported. Closed-loop walls, folded/multiple-branch
  contours, and curved layouts whose brick footprints cannot respect the region
  boundaries are rejected rather than producing an intruding layout.
- The continuous input surface stays visible behind the opening. The opening is
  an exclusion in the generated brick structure, not a hole in the NURBS.

Changing an opening invalidates analysis exactly like changing control points.
Worker snapshots include the opening definition, so editing during a solve cannot
change the job already running.

Verification: `python -m unittest surface_editor.test_openings -v` and
`python -m surface_editor.smoke_openings` from `gui/`. The smoke test uses the actual
GUI handles and full solver; it makes no paid AI requests.

## Connect an AI

Select OpenAI, Anthropic, Gemini, or OpenAI-compatible. Enter a **model ID available
in your account** and its API key. No model subscription or API credentials are included.
OpenAI uses the Responses API; OpenAI-compatible uses Chat Completions for local or
third-party endpoints. Choose a model that supports function calling.

The key field is masked and held in memory only. Alternatively set these environment
variables before launching (the program does not automatically load `.env` files):

| Provider | Key | Optional default model |
| --- | --- | --- |
| OpenAI | `OPENAI_API_KEY` | `OPENAI_MODEL` |
| Anthropic | `ANTHROPIC_API_KEY` | `ANTHROPIC_MODEL` |
| Gemini | `GEMINI_API_KEY` | `GEMINI_MODEL` |
| OpenAI-compatible | `AI_API_KEY` (optional for local servers) | `AI_MODEL` |

Base URLs are API roots, e.g. `https://api.openai.com/v1`, not full method URLs.
Remote endpoints require HTTPS; HTTP is allowed for localhost. Provider switches keep
the portable chat history and actual tool outcomes. Provider-specific reasoning blocks
and tool IDs are retained only within that provider's active tool loop.

Sending chat transmits its conversation and requested geometry/analysis data to the
selected endpoint. Geometry-only editing and analysis do not require any AI service.
Do not put credentials into chat or commit them to Git.

## What to try

- “Flatten the wall in its local depth direction.”
- “Make the top 20 percent narrower, keeping the height.”
- “Pull the middle outward by about 100 mm, keeping the boundary fixed.”
- “Twist gradually by 15 degrees from the bottom to the top.”
- “Smooth the interior control points without moving the perimeter.”
- “Move control point (2, 3) to [1200, 600, 900] mm.”
- “Give me a 10 by 8 control grid without changing the shape.”
- “Rebuild with 5 by 5 control points; allow at most 2 mm sampled deviation.”
- “Start a new flat wall, 2400 mm wide and 1200 mm high, with 8 by 6 points.”
- “Run a full analysis and then help me understand the problem areas.”
- After completion: “Which bricks moved most? What changes might reduce that?”

The agent can combine translate, scale, rotate, twist, bend, taper, local pull,
smooth, flatten, wave, and explicit control-point edits. It does not match prompts
against a hard-coded list of phrases. The LLM selects and composes tools from their schemas.
One batch is one undo step. You can continue dragging handles after an AI edit.
**Ctrl+Enter** sends a message. **Stop AI** prevents further tool execution; a pending
HTTP request can take up to its 90-second timeout to finish. Completed edits are retained.

## Geometry semantics and limits

- Geometry and displacement are assumed to be in **millimeters**, as in this repository.
- Local width is the horizontal projection of the loaded cage's U direction; height
  is world Z; depth is `height × width`. These axes stay fixed for the session and the
  agent can inspect them. “Outward” is relative to that depth vector, not the camera.
- Local displacement vectors are `[width, depth, height]`; explicit point positions
  are world XYZ. Local pull centers/radii use normalized control-grid coordinates.
- Pull amount controls strength; its Gaussian radius controls spread. The original
  **soft pull** slider controls neighbor movement during manual dragging.
- Edits change the existing control cage. Wave/bend amplitude is a cage displacement,
  not a guarantee of exact amplitude on the evaluated NURBS surface.
- Ordinary editing operations retain topology, degree, knots and weights. The
  separate `rebuild_surface` tool changes the grid: refinement preserves the original
  NURBS degree/shape, while rebuilding creates a non-rational cubic approximation.
  The UI and AI support 4–20 points in each direction. Trimming, holes and manual
  NURBS weight editing are not implemented.
- Edits are additive to the current state; this is not a persistent parametric modifier
  stack. Undo or explicit corrective changes support iteration.
- Numeric, index, shape and basic validity checks run before commit. They do not prove
  absence of folds/self-intersections or structural feasibility. The existing sampled
  section extractor remains approximate for complex folded surfaces.

## Analysis and feedback

**Geometry check** runs the existing L0 fit/repair without equilibrium. **Full analysis**
also requests the existing equilibrium and configured construction-stage checks. A
successful process can still return a wall that fails checks; the application preserves
those results for inspection.

Each run uses a detached geometry snapshot and a unique directory under `results/ai_*`.
Metadata stores the surface hash, revision, solver and mode; the result also contains
resolved analysis parameters. Manual editing stays responsive. The UI identifies
results as current, outdated, or unknown (legacy results without provenance). Restoring
exact geometry through undo can make a result current again.

Completion reports appear in chat. Ask a follow-up for AI interpretation; the tool gives
the model actual displacement, failures, interfaces, parameters and construction stages.
The model is instructed to distinguish hypotheses from solver evidence. The solver's
overall `ok` flag does not include construction-stage checks. A geometry-only run has
equilibrium **not checked**, not a confirmed structural failure or pass.

## Implementation map

| Module | Responsibility |
| --- | --- |
| `ai_app.py` | Qt window, chat, worker threads, GUI-thread tool dispatch |
| `providers.py` | Native API formats, authentication and HTTP errors |
| `agent.py` | Tool schemas, system instructions, bounded conversation/tool loop |
| `operations.py` | Deterministic geometry functions and argument validation |
| `session.py` | Revision checks, atomic edits, snapshots, analysis summaries |
| `topology.py` | Flat-wall creation, exact knot insertion, bounded grid fitting |
| `navigation.py` | Upright right-drag camera orbit and pan |
| `editor.py` | Existing viewport; accepts an injected plotter and shared edit callback |
| `refit.py` | Existing solver integration; unique result paths and output validation |

To add an operation, implement it in `operations.py`, describe its arguments in
`agent.py`, and add a geometric behavior test. No generated Python or shell commands
are evaluated. To add a provider, implement its request/reply/tool-result translation
in `providers.py`; geometry code remains independent.

## Verification

From `gui/`:

```powershell
..\.venv\Scripts\python.exe -B -m unittest surface_editor.test_ai -v
..\.venv\Scripts\python.exe -B -m unittest surface_editor.test_topology -v
..\.venv\Scripts\python.exe -B -m surface_editor.smoke_ai
..\.venv\Scripts\python.exe -B -m surface_editor.smoke_controls
```

The first suite is offline. The second briefly opens the real GUI, uses a scripted
provider to exercise the chat/tool bridge, checks manual editing and undo, saves a
temporary round-trip surface, runs the real geometry-only solver, and verifies
stale-result handling. It makes no paid API calls and leaves the original surface intact.
The older `smoke_test.py` requires historical result/GH fixtures absent from this clone.
`smoke_controls` tests actual Qt key/mouse events, upright orbit, text-vs-geometry undo,
and refinement/rebuilding with regenerated handles and topology-aware undo/redo.

Official API references:
- https://developers.openai.com/api/docs/guides/function-calling
- https://platform.claude.com/docs/en/agents-and-tools/tool-use/define-tools
- https://ai.google.dev/gemini-api/docs/function-calling
- https://qt.pyvista.org/usage.html
