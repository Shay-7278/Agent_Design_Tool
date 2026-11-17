# GH-derived IRB4600 brick assembly
## Runinng Commands 

docker compose exec robot-sim bash -lc "source /opt/ros/jazzy/setup.bash; source /ros2_ws/install/setup.bash; ros2 run abb_irb4600_gazebo execute_wall"

## Active contract

Only numeric [x, y, z, yaw] JSON rows are used, in metres/radians, relative to
MT_base at the milling table top. Full bricks are 190 x 90 x 57 mm: X is length,
Y is width. LL moves; RR stays visible and passive.

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

PLACED means the GH placement/release instruction completed. No settling,
displacement or structural-stability judgment blocks this animation.
Fresh timestamped actual poses are appended separately to
outputs/assembly_observations.jsonl; they are not a stability certificate.
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
