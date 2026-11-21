# GH-derived IRB4600 brick assembly
## Runinng Commands 

docker compose exec robot-sim bash -lc "source /opt/ros/jazzy/setup.bash; source /ros2_ws/install/setup.bash; ros2 run abb_irb4600_gazebo execute_wall"

## Active contract

Only numeric [x, y, z, yaw] JSON rows are used, in metres/radians, relative to
MT_base at the milling table top. Full bricks are 190 x 90 x 57 mm; confirmed
half bricks are 95 x 90 x 57 mm: X is length, Y is width. The original 221-row
demo uses 199 full bricks and 22 half bricks, with row order preserved.
LL moves; RR stays visible and passive.

Assembly visualization keeps waiting pickup bricks and completed placements
static. The selected pickup brick is activated as a dynamic body, physically
attached/carried, detached, then consolidated into fixed wall geometry at
its requested placement pose. Only the current brick needs dynamic contact
simulation. Completed geometry and full/half sizes are exported to
`outputs/assembly_manifest.json` after each placement or prefix prefill.

Stability assessment is a separate Gazebo run using individual dynamic
bricks and the same support geometry/materials. Movement warnings, missing
observations and time-budget exhaustion are reported without blocking the
assembly animation. Robot actuation and attachment failures remain errors.

The original GH archive and extracted Python/RAPID references are preserved.
config/gh_motion_profile.json records its SHA-256 and recovered values:
home TCP (0.5,3.45103,1.0) m; home X=+Y, Y=+X, Z=-Z; retract 200 mm;
rail offset -1000 mm; MoveJ v500; MoveL v50; cfx=0; WaitTime 1.
Re-read the profile with workcell_description/tools/read_gh_motion_profile.ps1.

GH motion order: home -> pickup lead -> pickup -> close/attach -> pickup lead ->
home -> placement lead -> placement -> release/open -> placement lead -> home.
Repeated identical home commands are skipped. MoveJ interpolates joints/rail;
MoveL interpolates position, quaternion and rail, then evaluates GH IK.
Smooth time parameterization uses GH nominal speeds and URDF actuator bounds;
ABB controller blending and exact real-controller timing are not emulated.

The fixed DH-to-URDF flange Rz(pi) conversion is applied once in the adapter,
separately from the measured tool TCP. The 495 mm base height is preserved.
No TAMP, collision-avoidance planner or new task-sequencing algorithm is added.

## Windows CMD

From Robot_Simulation\docker, rebuild and recreate:

```cmd
docker compose build
docker compose up -d --force-recreate
```

Wait for the pickup grid and controllers. In the same directory:

```cmd
docker compose exec robot-sim bash -lc "source /opt/ros/jazzy/setup.bash; source /ros2_ws/install/setup.bash; ros2 run abb_irb4600_gazebo execute_wall --preflight-only"
docker compose exec robot-sim bash -lc "source /opt/ros/jazzy/setup.bash; source /ros2_ws/install/setup.bash; ros2 run abb_irb4600_gazebo execute_one_brick"
```

For a separate full-wall run, recreate the world and replace execute_one_brick
with execute_wall. --placement-file /ros2_ws/inputs/FILE.json chooses an input;
--max-bricks N limits a test. Defaults include the fixed jig, brick SDF and MT frame.

221 bricks use batches 50,50,50,50,21. Refills use unique model names and retain
placed bricks. Automatic checkpoint/resume is not implemented; use a fresh world
after any failed or partial run.

To retain physics but disable the software-rendered GUI:

```cmd
set SIM_GUI=false
docker compose up -d --force-recreate
```

Use SIM_GUI=true before recreating to restore the viewer.

## Observations and limits

PLACED means the GH placement/release instruction and static consolidation
completed. Post-release physical poses are captured before consolidation
and appended separately to outputs/assembly_observations.jsonl. Static
prefix records are labelled as target geometry rather than measured dynamic
poses. No structural-stability judgment blocks this animation; its frozen
wall is not a stability certificate.
Actuation, TCP transfer and actual attach/release are still verified.

The agreed stroke leaves about 92.075 mm between pads for a 90 mm brick;
DetachableJoint supplies transport. Numerical geometry is not changed to hide
that gap. Original CAD/STL meshes remain; reduced simulation copies have a
geometry validation report. Gripper inertias remain provisional.
Some input poses overhang MT. Preflight does not certify support, swept-path
clearance or structural stability. Real robot/RAPID execution remains Phase II.

All tests live under src/abb_irb4600_gazebo/tests. The control and brick static
validators are included in that suite:

```bash
python3 -m unittest discover -s /ros2_ws/src/abb_irb4600_gazebo/tests -v
```

The tests include independent full-frame URDF checks and GH profile/sequence
checks. Mocked scheduling proves batch order, not a physical 221-brick build.
The inactive IRB120 workflow, competing loaders and duplicate converter were
removed, along with the obsolete diagnostic that required an active RR
controller. GH/RAPID reference sources remain unchanged.

## Execution verification — 2026-11-17

The six agreed corrections are implemented:

| Correction | Active files and resulting behavior |
|---|---|
| Restore GH motion authority | `config/gh_motion_profile.json`, `motion.py`, `execute_one_brick.py`: recovered home, 200 mm retract, -1000 mm track offset, v500/v50 and cfx=0; home at start, between pickup/placement and at end. |
| Reconcile flange conventions once | `irb4600_kinematics.py`: convert the GH flange's fixed Rz(pi) difference at the URDF boundary; preserve the 495 mm base offset and CAD tool transform. No new TCP offset was guessed. |
| Separate execution from assessment | `execute_one_brick.py`, `gz_brick_world.py`, `brick_lifecycle.py`: retain actuation/TCP/attachment checks; remove post-release settling and displacement gates; write timestamped observations separately. |
| Use one placement contract | `brick_lifecycle.py`, `startup_brick_spawner.py`: numeric-only XYZ/yaw; full bricks; placement relative to MT; unique pickup models replenished after each completed batch. |
| Reduce simulation load and derive timing | `tools.yaml`, optimized meshes, executor, Docker entrypoint/Compose: smaller meshes, GH speed-based timing within actuator bounds, and optional GUI. |
| Remove competing inactive paths | Removed the IRB120 executor/MoveIt client/converter, duplicate top-level tests, half-brick SDF, fabrication template, TC-master meshes and obsolete dual-controller motion diagnostic. Removed unused tool-change approach/slot commands from `fixtures.yaml`; fixture CAD poses remain. Updated package installation rules and stale documentation. |

Evidence is under `outputs/verification_2026-11-17/`:

- `unit_tests.log`: 23 passing tests, including independent full-frame URDF
  comparisons, both finger control mappings, watchdog cancellation and a mocked
  221-brick batch sequence (50,50,50,50,21).
- `wall_preflight.log`: all 221 brick routes passed using the live workcell
  transforms, including home, rail, MoveJ and sampled MoveL targets.
- `one_brick.log`: a real Gazebo cycle reached AVAILABLE, ATTACHED,
  TRANSPORTING, RELEASED, PLACED and `ASSEMBLY COMPLETE: 1 bricks`.
  Every arm endpoint passed the 3 mm TCP / 2 degree orientation checks.
- `one_brick_observations.jsonl`: released brick displacement was
  8.736e-7 m at the recorded simulation timestamp. This is a simulator
  observation, not a claim of equivalent real robot accuracy.
- `runtime_manifest.json`: the final cleaned image matched the live-tested
  image's 8 Python module ASTs, generated controlled URDF, all 36 mesh paths
  and 8 additional configuration/world/model/launch assets.

Test conditions: Gazebo physics remained at 0.001 s steps; collisions and
gravity remained enabled; the test GUI was disabled. The cycle took about
510 real seconds on this machine, with roughly 0.15–0.19x physics speed while
the user's existing GUI simulation also ran. The deployable image is built;
the user's existing container was left untouched and still uses its old image
until `docker compose up -d --force-recreate`.

Tested placement input SHA-256:
`ed71d7b395dde4d56be77285f3994c1427123402c201267ae0252191cc8198ab`.
The original GH source hash is stored in `gh_motion_profile.json`.

The entire 221-brick physical execution has **not** been run. Its route
preflight and mocked refill scheduling passed; neither proves all-wall contact
behavior or stability. The one-brick live test provides the execution evidence
for the corrected pipeline. No files were committed or pushed.

## Attachment correction and internal verification — 2026-11-17

The later wall run stopped while attaching `pickup_brick_010` (brick 11).
The adapter previously relied on a one-shot `attached` notification and a
15-real-second wait. It now reads Gazebo's actual DetachableJoint component
from `/world/irb4600_workcell/state`, matching the parent and child link IDs.
Notifications remain diagnostic. Attach/release requests can be retried
without loading a duplicate plugin. The joint wait permits 10 simulation
seconds and separately rejects a clock stalled for 30 real seconds.
The 15-real-second RPC transport bound remains a separate service-response
limit, not an attachment-completion deadline.

Fixed track origin/direction, reference-base basis and brick-to-flange mapping
are cached once per executor run. Rail displacement is still applied for every
target; measured TCP feedback remains live. Placement frames are transformed
once before planning. GH speeds, tool geometry and the physics timestep are
unchanged by this correction.

Evidence directory: `outputs/attachment_verification_2026-11-17/`.
The rebuilt image passes 36 tests. A live stress test passed 60 attach/release
cycles across 49 distinct bricks, including reattachment, with all attachment
state notifications deliberately suppressed. Every operation was confirmed by
actual component readback; the detailed record is `attachment_stress.jsonl`.
These are protocol tests, not a complete wall build.

The current 221-brick execution test uses the default smaller assembly scene,
the original numeric input, full bricks, 0.001 s physics steps, target real-time
factor 1.0, collisions and gravity, with the GUI disabled. Its full route
preflight passed. The full execution attempt completed nine brick cycles,
then stopped on brick 10's placement descent: controller success was followed
by a measured TCP miss greater than 3 mm. Brick 10's attachment and transport
had passed. This run is **not a successful 221-brick execution**.

### Remaining blocker: overlapping placement geometry

Using only the numeric input and the active 190 x 90 x 57 mm Gazebo box model,
targets 9 and 10 intersect at the same height. In the preserved failed scene,
brick 10 still had a physical attachment joint and the TCP eventually reached
the target, but brick 9 had been pushed approximately 266 mm from its target.

A clearance replay of the same endpoint passed. For a controlled contact
replay, all first-nine brick target poses were restored in the isolated test
world and their positions verified within 0.001 mm before descent. That
reproduced the problem: after controller success, TCP error was 48.09 mm
initially, 47.68 mm after 0.20 simulation seconds, and 44.40 mm after 2.00
simulation seconds. Joint 2 error was about 0.0365 rad, within the controller's
existing 0.1 rad goal tolerance. Thus the controller's success criterion and
the stricter TCP check differ; in this contact case the TCP check correctly
prevented continuing with a missed pose. It must not be weakened to disguise
the interference.

See `wall_execution.log`, `wall_observations.jsonl`,
`stopped_wall_diagnostics.json`, `clearance_completion_trace.json`,
`collision_baseline.json`, `collision_completion_trace.json` and
`placement_collision_diagnosis.png` in the evidence directory. Supplemental
replays changed test-world poses only; the input JSON, model sizes, controller
tolerances, TCP and motion speeds were not changed.

The pose-only demo/numeric files cannot identify a half brick or establish
the intended dimensions of an overlapping brick. Matching original wall
geometry or per-brick dimensions are needed before correcting model selection
and certifying a complete wall execution. No `physics_check` logic was used
for this diagnosis. The attachment/caching corrections are tested and built;
the complete wall remains blocked by the geometry issue.

## Start at brick 11 with a prefilled prefix

`execute_wall --start-brick 11` first places source rows 1-10 directly at
their MT-relative targets, then executes the original pickup/placement routes
from row 11 onward. It relocates the existing startup models rather than
creating duplicate bricks. Row 11 remains `pickup_brick_010`, using its
original pickup slot; later batches still begin at rows 51, 101, 151 and 201.
Use a fresh world. The initial pickup batch is checked before relocation.

Earlier rows are logged as `PREFILLED`, separately from robot-executed `PLACED`
rows, with measured poses and displacement. Full-brick dimensions, gravity,
collisions, GH speeds and TCP tolerances remain active. This mode bypasses
the first ten robot cycles; it does not repair their overlapping geometry or
provide a valid half-brick mapping. Prefilled bricks may move under physics.

From the Windows CMD terminal in `Robot_Simulation\docker`, after rebuilding:

```cmd
docker compose up -d --force-recreate
docker compose exec robot-sim bash -lc "source /ros2_ws/install/setup.bash && ros2 run abb_irb4600_gazebo execute_wall --start-brick 11"
```

Wait for startup to report `spawned 50 pickup bricks` before the executor.
For a one-cycle diagnostic, append `--max-bricks 11`; the limit caps the input
row number, so start 11 / max 11 executes only row 11. `--preflight-only`
checks the remaining robot routes without preplacing any models.

Internal test of this mode: all 42 unit/configuration tests passed in the final
image. A fresh Gazebo run prefilled rows 1-10 and reached brick 11's ATTACHED
and TRANSPORTING states. Its physical attachment was confirmed with no state
notification received. It then stopped at the placement descent's existing
3 mm TCP check; release and cycle completion did not occur. A later readback
found the tool and carried brick at the target, with the attachment still
present. This later convergence does not establish the error at the instant
of the failed check or prove its cause. Prefilled bricks 9 and 10 had moved
about 32 mm and 14 mm respectively. The next failure is therefore not an
attachment timeout, and prefill does not make the wall physically correct.
Evidence: `outputs/prefill_verification_2026-11-17/brick_11_execution.log`,
`assembly_observations.jsonl`, `stopped_brick_11_readback.json` and the two
unit-test logs. No successful brick-11 or complete-wall execution is claimed.

## Measured tool-arrival correction

The executor now replaces the single check after 0.2 simulation seconds with
a bounded measured-arrival wait. Following controller success, fresh TCP
feedback must remain within 3 mm and 2 degrees for 0.1 simulation seconds.
The arrival budget is 10 simulation seconds; a clock stopped for 30 real
seconds or a reset aborts the wait. Slow but advancing simulation clocks do
not consume the simulation-time budget faster. Controller tolerances, GH
motion speeds, collision geometry and physics settings are unchanged.

Each arm endpoint records the measured error over time and ARRIVED/FAILED
outcome in `outputs/assembly_observations_tcp_arrival.jsonl`. A timeout
reports the final position/orientation error. The next instruction and
planned execution state advance only after measured arrival. This is a ROS
execution adapter check, not an extracted GH structural-stability test.

The final source passes 50 unit/configuration tests, including late arrival,
position/orientation limits, stale feedback, slow/stopped/reset clocks and
preserving the previous execution state on failure. Live-test evidence is
under `outputs/tcp_arrival_verification_2026-11-17/`; the earlier failed
prefill test above remains historical evidence from before this correction.

The corrected fresh-world test completed brick 11's full physical cycle:
ATTACHED, TRANSPORTING, RELEASED, PLACED and return home. Final log:
`ASSEMBLY COMPLETE: 1 robot-placed bricks; 10 PREFILLED`. The final installed
image also passed all 50 tests. At the previously failing placement descent,
measured position error was 11.288 mm after 0.185 simulation seconds, then
0.185 mm with 0.0083 degrees error when stable arrival was confirmed after
0.341 simulation seconds. Release was confirmed by Gazebo component readback.
The measured completion wait therefore resolves this brick-11 premature
abort without relaxing the 3 mm / 2 degree thresholds. This verifies that
cycle, not a complete 221-brick wall or structural stability. The missing
per-brick size mapping and the prefilled geometry overlap remain unresolved.

The updated local image is already built. Start a fresh Compose world, wait
for `spawned 50 pickup bricks`, then use the CMD command in the preceding
start-at-brick-11 section without `--max-bricks` to attempt the remaining wall.

## Viewer load and motion speed

The Compose desktop now defaults to `SIM_RESOLUTION=1280x720` and
`GUI_FPS=20`. The viewer uses Qt's basic render loop with a 50 ms update-idle
interval as a rendering-rate target. Actual frame rate depends on the
renderer and event scheduling. Scene/light shadows are disabled. Colour and
directional shading remain. The physics timestep, gravity, collision shapes,
material properties and robot controller configuration are unchanged.
The 1280x720 desktop has 56% fewer pixels than the previous 1920x1080 default.

`execute_wall --speed-scale 2` requests twice the original GH linear and
angular arm motion speeds. Spatial waypoints, home/retract/rail logic and
arrival checks are unchanged. Existing URDF joint/rail velocity limits and
the 0.5 simulation-second minimum segment duration still bound trajectories,
so not every move is twice as fast. Gripper durations and GH waits remain
unchanged. Default `--speed-scale 1` retains the original timing. This option
does not change Gazebo's real-time factor or physics timestep.

The new image is built separately; an already running executor does not
change speed automatically. Applying the new viewer settings requires a
fresh Compose world, which resets the current scene. From Windows CMD in
`Robot_Simulation\docker`:

```cmd
docker compose up -d --force-recreate
docker compose logs -f robot-sim
```

After `spawned 50 pickup bricks`, Ctrl+C exits log following. Then:

```cmd
docker compose exec robot-sim bash -lc "source /ros2_ws/install/setup.bash && ros2 run abb_irb4600_gazebo execute_wall --start-brick 11 --speed-scale 2"
```

The final image passes 55 unit/configuration tests. Speed tests verify that
an unconstrained 200 mm MoveL is shortened from 7.5 to 3.75 simulation
seconds with identical waypoints, rotational timing scales, rail speed
limits still constrain all sampled intervals, and invalid speed values are
rejected. The optimized viewer launched successfully and was visually
checked at 1280x720. Evidence is under
`outputs/speed_verification_2026-11-17/`. The existing user wall run was not
restarted or modified during these checks.

The live `--speed-scale 2` test completed brick 11's pickup, physical
attachment, transport, placement, verified release and return home. Final
log: `ASSEMBLY COMPLETE: 1 robot-placed bricks; 10 PREFILLED`. The placement
arrival check passed at 0.054 mm / 0.0024 degrees after 0.547 simulation
seconds. The GUI smoke test was completed first, then only the isolated
test viewer was closed to reduce concurrent rendering load during the
physical cycle. The user's original viewer and executor stayed running.
These shared-machine runs are not a controlled wall-clock speed benchmark;
2x refers to requested motion speeds, not a guaranteed 2x real-time factor.
The full wall and missing half-brick mapping remain unverified.

Viewer scheduling uses supported Qt settings: see
[QWindow update scheduling](https://raw.githubusercontent.com/qt/qtbase/5.15/src/gui/kernel/qwindow.cpp)
and the [Qt basic rendering loop](https://raw.githubusercontent.com/qt/qtdeclarative/5.15/src/quick/scenegraph/qsgrenderloop.cpp).

## Confirmed half bricks in the 221-row demo

Keep the original `inputs/brick_frames_numeric.json`. No placement row is
deleted or renumbered. The user-confirmed 1-based half-brick rows are:

```text
9, 17, 26, 34, 43, 51, 60, 68, 78, 87, 97, 107,
118, 128, 140, 151, 163, 173, 184, 193, 203, 212
```

`config/brick_sizes.json` binds these rows to the confirmed placement content.
Renaming or reformatting that input preserves the mapping. Different pose
sequences retain the existing full-brick default; their sizes require their
own confirmed profile. The input format remains four numbers XYZ/yaw.

Startup and every refill use the same model selector and absolute brick IDs.
The existing 50 pickup centres accommodate both sizes; no extra slots are
needed. The half-brick SDF has a 95 x 90 x 57 mm box, mass 0.341145 kg and
recalculated box inertia at the existing density. Friction/contact settings,
colour, height, width, TCP and finger stroke are unchanged. Prefilled rows
also keep their correct model sizes. The first 50 supply bricks contain five
halves: source rows 9, 17, 26, 34 and 43.

All 66 unit/configuration tests pass. A geometric check of all 221 placements
with these sizes found no same-level solid intersections. These corrections
supersede the earlier full-only and missing-size limitations documented above;
assembly now avoids the growing dynamic-stack workload through the separate
visualization/assessment workflow below. Live half-model evidence is under
`outputs/half_brick_verification_2026-11-17/`.

## Two-phase assembly visualization and stability assessment

The current assembly mode keeps the supply display static, activates only
the selected full/half pickup brick, physically carries it with a verified
DetachableJoint, verifies release, then consolidates it into static wall
geometry at the requested JSON pose. The old static support model is kept
until its replacement is verified. Static status is read from Gazebo's ECM;
the Scene message's default false flag is not a reliable static-state check.
Read-only state/scene requests have one bounded retry; entity mutations are
not blindly repeated. Startup waits for all expected pickup models before
validating or changing the fresh scene.

Every completed placement/prefill exports `outputs/assembly_manifest.json`
with world poses, model paths and full/half dimensions. Post-release physical
observations are captured before the individual model is removed. Static
targets are not reported as measured physical accuracy. The selected robot
still has the same actuator, TCP and attachment checks. Assessment warnings
do not participate in those checks and never stop assembly.

After recreating the world and waiting for `spawned 50 pickup bricks`, the
assembly command remains:

```cmd
docker compose exec robot-sim bash -lc "source /ros2_ws/install/setup.bash && ros2 run abb_irb4600_gazebo execute_wall --start-brick 24 --speed-scale 2"
```

Run the separate assessment on an exported snapshot with:

```cmd
docker compose exec robot-sim bash -lc "source /ros2_ws/install/setup.bash && ros2 run abb_irb4600_gazebo assess_stability"
```

Assessment starts its own Gazebo world/transport partition with individual
dynamic full/half models, the calibrated MT collision mesh, platform and
unchanged material/physics timestep. It observes up to 10 simulation seconds
with a 60-real-second maximum. `OBSERVED`, `WARNING` and `INCOMPLETE` outcomes
are written to `outputs/stability_report.json`; all exit zero and never
modify or interrupt the assembly scene. A warning is still recorded, not
misrepresented as a passed stability test. Final-geometry observation alone
does not certify construction-stage or real-robot stability.

Internal evidence is under `outputs/static_assembly_verification_2026-11-17/`.
The integrated suite passes 117 tests, including correct full/half geometry,
all 221 batch identities, startup readiness, release-before-consolidation,
manifest contents, a 221-link SDF beyond the command-line argument limit,
non-blocking assessment warnings/time budgets, delayed clock delivery after
prefill, and frozen/backward tool-feedback rejection. New trajectory and GH
wait budgets start from Gazebo's current clock rather than a ROS clock sample
cached before blocking scene operations. Tool arrival still requires 3 mm /
2 degree accuracy held across 0.1 simulation seconds of advancing TF feedback;
it does not compare the absolute ages of independently delayed callbacks.
Native pose/clock subscriptions are rate-limited to bound Python callback
work as the static wall grows. Physics and controller update rates are unchanged.
Scene confirmation waits for the complete expected link list, since Gazebo can
publish the model before all its links. Its bounded publication wait is 60 real
seconds; the previous 10-second window failed under shared-machine load even
though the complete static model appeared afterward. Transport subscriptions are explicitly
unsubscribed and allowed to finish before Python shuts down.

Course boundaries: rows 1–8 form course 1, rows 9–17 form course 2, and
row 18 starts course 3. `--start-brick 18` therefore preplaces exactly the first
two courses, including half bricks 9 and 17, and executes rows 18–221. The
previous session wrote `outputs/assembly_from_18.log`. It completed bricks
18–23 and stopped before pickup of brick 24 after a lost create-service reply.
Do not launch a second executor into an already used world. The current restart
is `--start-brick 24`, which preplaces rows 1–23 in a fresh world and executes
rows 24–221. From CMD in `Robot_Simulation\docker`, apply the already built image:

```cmd
docker compose up -d --force-recreate
docker compose logs -f robot-sim
```

After `spawned 50 pickup bricks`, press Ctrl+C to leave log following and run
the brick-24 executor command above. No rebuild is required for the local image
already prepared in this session.

Live evidence: bricks 17 and 18 completed physical cycles in the earlier
two-brick integration test. The final-course fixture reused a read-back-verified
220-link static prefix, then ran the production robot operations for brick 221:
pickup, attachment, carry, release, static consolidation, retract and return home.
It exited zero with `ASSEMBLY COMPLETE: 1 robot-placed bricks; 220 PREFILLED`.
Its physical post-release centre error was 0.000000912 m. All remaining targets
11–221 passed preflight with a clean exit, and the final image passed all 95
tests. These checks do not mean that all 221 robot cycles have run consecutively.
The brick-18 session used its own fresh prefill and completed six consecutive
physical cycles before the create-reply failure. Its Gazebo viewport was
inspected in `outputs/assembly_from_18_visual.png`.

The independent 221-brick assessment exited zero as `INCOMPLETE`: its short
12-real-second smoke-test budget elapsed before observation began. This confirms
the non-blocking result path, not structural stability. Earlier live assessment
fixtures separately verified supported bricks and falling-brick warnings.

Create-reply correction: a service timeout now means an uncertain result,
not proof that creation failed. Create requests are sent once. Activation then
requires a new entity ID, an explicitly false ECM Static flag, the selected
full/half collision dimensions, a fresh pose belonging to that new ID, and a
pickup position/orientation match within 5 mm / 2 degrees. Collision geometry
comes from ECM, since the live Scene response omits collisions. Explicit
rejection, wrong size, static/missing state and stale/wrong poses still fail.
Supply and combined-wall creation likewise proceed to their existing readback
after an ambiguous timeout rather than blindly repeating the mutation.

The create-reply image passed 106 tests. An isolated live Gazebo test intentionally
discarded successful create replies for a full brick, a half brick and static
consolidation; all recovered through verified readback, with exactly one create
per operation. Evidence: `create_recovery_tests.log`, `create_recovery_live.log`
and `create_recovery_image_build.log` in the static-assembly verification folder.
This tests the observed transport failure; the entire remaining wall has not
yet been executed consecutively.

Pickup height correction: the source CAD export keeps its original 86.5 mm
centre Z, but runtime pickup poses use the current collision support and brick
height. The platform top is 44 mm (`fixtures.yaml`); both full and half boxes
are 57 mm high. Their centre therefore spawns at 72.5 mm, with no 14 mm float
above the platform. `pickup_pose_on_platform` supplies this same calculation to
startup, static refills and executor pickup targets. It reads collision height
from the selected model and preserves exported X/Y/orientation. Source pickup
and placement JSON files are unchanged. The 5 mm / 2 degree activation limits
and all TCP arrival limits are unchanged.

The final height-corrected image passes 110 tests, including all 50 actual
exported pickup slots, full/half startup requests, collision-derived height,
and static-refill height. Evidence is under
`outputs/pickup_height_verification_2026-11-17/`.
The isolated actual-workcell test ran the normal executor with
`--start-brick 24 --max-bricks 24 --speed-scale 2`. It prefilled rows 1–23,
then completed brick 24's physical pickup, verified attachment, carry, placement,
verified release, static consolidation, retract and return home. The process
exited zero with `ASSEMBLY COMPLETE: 1 robot-placed bricks; 23 PREFILLED`.
The log is `brick_24_cycle.log` in that evidence directory. This verifies brick
24 with the real platform geometry; it does not claim the remaining wall has
completed. The user's stopped scene was left unchanged during verification.

## Persistent Gazebo command handling

The current image uses one persistent native Gazebo Transport client per
spawner/executor process. It no longer launches a `gz service` process for each
request. The helper is built by the Dockerfile from
`src/abb_irb4600_gazebo/native/` and installed in the existing package. The
Python adapter exchanges framed protobuf messages with it. A damaged/partial
reply stream is discarded before another request can use it.

Mutation requests are sent once. Both positive acknowledgements and ambiguous
timeouts proceed to operation-specific state confirmation. Creation checks
identity, mode, collision dimensions and fresh pose. Attachment loading keeps
a pending state and confirms the exact physical parent/child fixed joint
before recording success, without loading a second plugin after a lost reply.
Removal addresses and confirms disappearance of the captured entity ID.
Pose updates require fresh feedback from the same ID. Startup and refill
spawning use this same adapter. Readback timeouts retry within one operation
deadline; a clock reset stops the operation. Static-wall metadata and the
manifest advance only after replacement/removal confirmation.

Evidence is under `outputs/service_layer_verification_2026-11-17/`:

- `final_image_tests.log`: all 117 tests pass in the deployment image.
- `native_build_check.log`: helper builds from the standard ROS environment.
- `third_course_cycle.log`: normal physical brick-18 cycle completes with 17 prefilled.
- `mutation_faults.log`: 1,000 alternating scene/state requests use one helper
  PID. Live full/half creation, activation, pose update, attachment and removal
  recover after deliberately discarded successful replies; no duplicate plugin loads.
- `brick24_fault_cycle.log`: a verified 23-member test prefix followed by a full
  physical brick-24 cycle, discarding every successful mutation reply during
  the cycle. Three removals, two creates and one plugin load are each sent once.
  The cycle exits zero with `ASSEMBLY COMPLETE: 1 robot-placed bricks; 23 PREFILLED`.

The requested restart is brick 24, within course 3 (rows 18-25). A fresh world
preplaces rows 1-23 and executes 24-221. After recreating the container and
waiting for `spawned 50 pickup bricks`, use the brick-24 command above. The
local deployment image has already been built; a rebuild is not needed.
These physical and fault tests do not claim that the remaining wall has
completed as one uninterrupted run.
