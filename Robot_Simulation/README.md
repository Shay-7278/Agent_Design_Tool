# GH-derived IRB4600 brick assembly

## Active contract

Only numeric [x, y, z, yaw] JSON rows are used, in metres/radians, relative to
MT_base at the milling table top. Full bricks are 190 x 90 x 57 mm: X is length,
Y is width. LL moves. The default assembly workcell contains LL, its track and
gripper, MT, the pickup platform, ground and bricks.

## Smaller workcell

`simulation.launch.py` now passes `abb_irb4600_assembly.gazebo.xacro` by default.
It reuses the full workcell's calibrated transforms and detailed LL robot,
track and gripper geometry. The RR robot/track, MHS, saw, tool stations and
decorative IRB7600 are omitted. Nine control joints remain: track, six arm
joints and both gripper fingers. `assembly_controllers.yaml` keeps the same LL
controller settings; `assembly_initial_positions.yaml` contains only those
nine joints.

Only the MT visual uses a reduced mesh (145,232 -> 6,000 triangles). Its
original collision geometry and top surface are unchanged. The source CAD
mesh is retained; `meshes/optimized/fixtures/MT_base.validation.json` records
the reduction and geometry checks. Robot and track meshes are not simplified.
Physics timestep, gravity, contact settings and GH motion timing are unchanged.

The full scene remains available with
`xacro_file:=/ros2_ws/install/abb_irb4600_gazebo/share/abb_irb4600_gazebo/urdf/abb_irb4600_workcell.gazebo.xacro`.
Rebuild and recreate the container to load the new default scene. Recreating
starts a fresh world; it does not resume a failed assembly.

Performance claims below from 2026-11-17 describe the earlier full workcell.
The smaller scene needs its own runtime benchmark.

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

## Full-workcell execution verification — 2026-11-17

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
