# Grasshopper task and motion framework extraction

This directory is a reference-only extraction from
`ARC574_F26_RoboticBrickAssembly_IRB4600_Rhino8_Template.gh`. The extracted
files are not imported by ROS and do not change the Gazebo simulation.

The extraction is reproducible with
`tools/extract_grasshopper_reference.ps1`. `manifest.json` records the source
index, SHA-256 digest, and whether each embedded script was extracted.

## Scope decision

Embedded scripts 1-20 were extracted verbatim. Scripts 21-24 were inventoried
but deliberately not extracted because they implement brick bond/support graph
analysis and iterative joint relaxation. That graph logic is deferred and is
not an input to the first pick-and-place simulation.

## Classification

| Script(s) | Role | ROS disposition |
|---|---|---|
| `01_py3_01.py` | Select matching chunks of jig and wall bricks | Keep as task-data preparation reference |
| `02_py3_02.py` | Generate pickup TCP and world-Z retract frames | Port after coordinate/unit verification |
| `03_py3_03.py` | Generate placement TCP and world-Z retract frames | Port after coordinate/unit verification |
| `04_py3_04.py` | IRB4600 RAPID data types, FK, IK, reachability, TCP/flange conversion, and track placement | Candidate motion-core reference; validate numerically against URDF |
| `05_define_tool_01.py` | Convert a Rhino tool plane/load into ABB `ToolData` | Port the data contract, not Rhino types |
| `06_movej_01.py`, `13_movej_02.py`, `15_movej_03.py`, `16_movej_04.py`, `19_movej_05.py` | Identical `MoveJ` instruction builders | Deduplicate to one ROS implementation |
| `12_movel_01.py`, `14_movel_02.py`, `17_movel_03.py`, `18_movel_04.py` | Identical `MoveL` instruction builders | Deduplicate to one ROS implementation |
| `07_visualizer_01.py` | Rhino preview interpolation, robot/tool mesh transforms, and simulated brick display state | Split semantics from Rhino-only rendering |
| `08_py3_05.py` | Return Rhino's current working folder | Do not port |
| `09_py3_06.py` | RAPID module/file exporter | Do not use as the ROS kinematics source: it still declares `DH_IRB120` |
| `10_py3_07.py` | Sort wall bricks into courses | Optional task-data preprocessing; not trajectory planning |
| `11_py3_08.py` | Produce simplified collision-brick geometry | Reference for brick occupancy only |
| `20_py3_09.py` | Convert a truthy input to a Boolean output | Do not port |

## Task sequence recovered from the graph

1. Start/home state (the graph's Task 1 label is not present in the extracted panels).
2. `MoveJ` to pickup lead-in.
3. `MoveL` to pickup TCP frame.
4. `WaitTime 1`.
5. Close the LL gripper (`Reset LL_GRIPPER`).
6. `WaitTime 1`.
7. `MoveL` to pickup lead-out.
8. `MoveJ` home.
9. `MoveJ` to placement lead-in.
10. `MoveL` to placement TCP frame.
11. `WaitTime 1`.
12. Open the LL gripper (`Set LL_GRIPPER`).
13. `WaitTime 1`.
14. `MoveL` to placement lead-out.

## Inputs to the first ROS/Gazebo system

### Static model inputs

- LL and RR IRB4600/IRBT4004 URDF geometry and joint limits.
- Workcell, fixture, and platform transforms.
- `TC_master` and `tool_attachment_frame` transforms.
- Brick-gripper body and open-state left/right finger meshes.
- Finger stroke: 0.008875 m per finger.
- Gripper TCP plane from Grasshopper, pending CAD-frame equivalence proof.
- Brick dimensions, pickup poses, placement poses, and collision geometry.

### Runtime task inputs

- Active robot: LL. RR remains visible and stationary.
- Ordered pickup bricks and corresponding ordered wall bricks.
- `z_retract` lead-in/lead-out distance.
- LL robot-root plane and track offset.
- Tool frame and tool load.
- Robot configuration branch (`rob_conf`).
- Motion speed and zone values.
- Home joint target.
- Gripper open/close commands and wait duration.

### Outputs

- LL seven-axis trajectories: track plus six robot joints.
- One commanded gripper joint and one mirrored finger state.
- Brick attachment/release state.
- Gazebo visual and physical state.
- Optional RAPID text as a reference output, not the Gazebo control path.

## Rhino-only visualization code

The following behavior in `07_visualizer_01.py` is specific to Rhino preview
and should not be copied into the ROS renderer:

- `Rhino.Geometry` planes, transforms, curves, meshes, and quaternions.
- Grasshopper data-tree conversion through `ghpythonlib.treehelpers`.
- Transforming copied robot and tool meshes for viewport display.
- A time slider driving preview interpolation.
- Rebuilding jig/wall/gripped-brick display collections for each preview frame.

The reusable semantics inside the same file are:

- Classifying `MoveJ`, `MoveL`, and `MoveAbsJ` segments.
- Cartesian interpolation and quaternion slerp for `MoveL`.
- Joint/IK interpolation behavior used for `MoveJ` preview.
- Track interpolation.
- `Reset LL_GRIPPER` as pickup and `Set LL_GRIPPER` as release.
- The state transition identifying which brick is held or placed.

Gazebo replaces the mesh-preview portion. A ROS task executor retains the
instruction and state-transition portion.

## Historical extraction checklist (not the current runtime contract)

The list below records the original extraction stage. The current runtime uses
direct mounting without TC_master; see Robot_Simulation/README.md and
config/gh_motion_profile.json for the active workflow. Embedded source and RAPID
syntax are preserved as historical references, not imported as runtime modules.

1. Confirm SI conversion: Rhino/STL millimetres to URDF metres.
2. Confirm LL track zero, positive direction, and 0-4 m range against the GH
   convention of motion along negative `robot_root.YAxis`.
3. Compare GH FK/IK with the generated URDF at sampled joint configurations.
4. Prove the complete flange -> `TC_master` -> tool attachment -> gripper TCP
   transform; do not add the TC-master offset twice.
5. Verify the open-state finger meshes do not initially intersect the body and
   that inward travel of 8.875 mm per side is collision-valid.
6. Add body/finger collision meshes when supplied and verify inertial values,
   joint effort, velocity, damping, and friction.
7. Verify robot/track stability, joint-limit behavior, and stationary RR state.
8. Validate brick mass/inertia, contact friction, and collision dimensions.
9. Use deterministic attach/release for the first visualization baseline;
   contact-only grasping can be evaluated separately.
10. Check every sampled `MoveL` pose for IK continuity and joint/track limits.

## Joining the two systems after validation

Keep the task result and the simulator separated by a narrow command boundary:

1. A task-data adapter converts extracted brick/TCP targets from Rhino units
   and frames into ROS poses.
2. A motion adapter converts each accepted `MoveJ`/`MoveL` instruction into a
   timestamped seven-joint LL trajectory.
3. The existing `LL_manipulator_controller/follow_joint_trajectory` action
   executes that trajectory in Gazebo.
4. A gripper action commands the two 8.875 mm finger motions.
5. A grasp-state node attaches the selected brick after confirmed close and
   releases it after confirmed open.
6. Joint states, TCP error, track error, gripper state, and brick pose provide
   execution feedback. The next task step starts only after the current result
   is accepted.

This boundary allows the initial extracted planner to be replaced later by
MoveIt/OMPL without rewriting the task sequence or Gazebo model.
