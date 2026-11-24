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
uv run --project .. python -m surface_editor --ai
```

Optional: append a surface path and/or `-r path_to_result.json`.
The default is `Reference Surface_1.3dm`.

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
- “Run a full analysis and then help me understand the problem areas.”
- After completion: “Which bricks moved most? What changes might reduce that?”

The agent can combine translate, scale, rotate, twist, bend, taper, local pull,
smooth, flatten, wave, and explicit control-point edits. It does not match prompts
against a hard-coded list of phrases. The LLM selects and composes tools from their schemas.
One batch is one undo step. You can continue dragging handles after an AI edit.
**Ctrl+Enter** sends a message. **Stop AI** prevents further tool execution; a pending
HTTP request can take up to its 300-second timeout to finish. Completed edits are retained.

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
- The current reference has a 6 × 6 grid, too coarse for fine detail such as several
  waves along the wall. **Add control points…** (toolbar), or the AI's `add_control_points`
  tool, inserts knots along the wall or up its height. The surface shape stays exactly the
  same, the edit is undoable, and **Reset surface** returns to the loaded grid. Aim for about
  4 control points per wave cycle. Degree and weights stay unchanged. No arbitrary topology
  creation, trimming, holes, or NURBS weight editing is implemented. Use flatten(depth)
  for a planar wall.
- Wave along the width accepts an optional `fade_height` (fraction of wall height): full
  strength at the base, fading to zero at that height, for base-only undulation.
- AI edits must keep the wall base level (one shared height for the base row of control
  points), so brick courses stay horizontal. Edits that tilt the base are rejected;
  manual dragging is not restricted.
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
..\.venv\Scripts\python.exe -B -m surface_editor.smoke_ai
```

The first suite is offline. The second briefly opens the real GUI, uses a scripted
provider to exercise the chat/tool bridge, checks manual editing and undo, saves a
temporary round-trip surface, runs the real geometry-only solver, and verifies
stale-result handling. It makes no paid API calls and leaves the original surface intact.
The older `smoke_test.py` requires historical result/GH fixtures absent from this clone.

Official API references:
- https://developers.openai.com/api/docs/guides/function-calling
- https://platform.claude.com/docs/en/agents-and-tools/tool-use/define-tools
- https://ai.google.dev/gemini-api/docs/function-calling
- https://qt.pyvista.org/usage.html
