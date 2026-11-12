"""
construct_using_validated.py
============================
Simulation pick-and-place from a JSON file of brick poses.

Workflow
--------
1. Load brick poses in world coordinates from a JSON file.
2. Plan and execute each pick-and-place with the configured grasp via MoveIt.
3. After each successful placement, register the placed brick as a collision
   object so subsequent plans avoid it.

Usage (inside Docker with MoveIt running)
------------------------------------------
  # Plan and execute in Gazebo:
  python3 scripts/construct_using_validated.py --poses-file inputs/poses.example.json --sim

CLI arguments
-------------
  --poses-file: world-frame input JSON (required)
  --sim       : execute trajectories in Gazebo (default: plan only)
  --hover-z  : additional Z height for pre/post grasp hover in metres (default: 0.12)

Grasping Poses
--------------
Extracted from src/grasping_poses/grasping_poses.3dm.
Each entry is a 4x4 homogeneous matrix T_grasp_in_brick:
  T_tcp_world = T_brick_world @ T_grasp_in_brick

The brick_pose reference frame in the Rhino model:
  origin  : [0, 0.42, 0.0315]
  X-axis  : [+1,  0,  0]   (longest brick dimension)
  Y-axis  : [ 0, -1,  0]   (medium brick dimension, note flipped)
  Z-axis  : [ 0,  0, +1]   (shortest brick dimension / vertical)
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import time
from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation as R

# ---------------------------------------------------------------------------
# ROS 2 / MoveIt availability guard -- allows offline syntax-check / dry-run
# ---------------------------------------------------------------------------
try:
    import rclpy
    # Import the planner client from the sibling script
    sys.path.insert(0, os.path.dirname(__file__))
    from moveit_client import PlanAndExecuteClient

    HAVE_ROS2 = True
except ImportError:
    HAVE_ROS2 = False
    print(
        "WARNING: rclpy / moveit_client not importable. "
        "Script will validate logic only (planning calls are no-ops)."
    )

# ---------------------------------------------------------------------------
# Pose conversion utility (Brick class)
# ---------------------------------------------------------------------------
sys.path.insert(0, os.path.dirname(__file__))
from pose_conversion import Brick


# ===========================================================================
# Constants
# ===========================================================================

BRICK_SIZE_XYZ: tuple[float, float, float] = (0.051, 0.023, 0.014)

# Relative transforms from the brick center frame to each gripper TCP pose.
# Derived from src/grasping_poses/grasping_poses.3dm (see module docstring).
#
# Convention:
#   - Axis lengths in Rhino: longest=X (0.020 m), mid=Y (0.015 m), shortest=Z (0.010 m)
#   - The TCP frame Z-axis is built as the *shortest* Rhino axis so it
#     aligns with the gripper approach direction (pointing toward the brick).
#   - T_tcp_world = T_brick_world @ T_GRASP_IN_BRICK
#
# Data extracted by the Python script run on the .3dm file:
#   brick_pose T:
#     [[ 1,  0,  0,  0   ],
#      [ 0, -1,  0,  0.42],
#      [ 0,  0,  1,  0.0315],
#      [ 0,  0,  0,  1   ]]
#
# Each T_grasp_in_brick = T_brick_inv @ T_grasp_world


def _t(rot_cols: list[list[float]], pos: list[float]) -> np.ndarray:
    """Build a 4x4 homogeneous matrix from column vectors and position."""
    T = np.eye(4)
    T[:3, :3] = np.column_stack(rot_cols)
    T[:3, 3] = pos
    return T


# Fixed orthogonal top-down grasp. The TCP Z axis points straight down the
# brick's local Z axis; X follows brick X and Y is reversed to keep the frame
# right-handed.
T_GRASP_IN_BRICK = _t(
    rot_cols=[
        [1.0000, 0.0000, 0.0000],
        [0.0000, -1.0000, 0.0000],
        [0.0000, 0.0000, -1.0000],
    ],
    pos=[0.0000, 0.0000, 0.0000],
)

SUPPLY_BRICK_NAMES: tuple[str, ...] = tuple(f"brick_{i:02d}" for i in range(20))
MAX_BRICKS = len(SUPPLY_BRICK_NAMES)
# The world file starts bricks at 0.031 m, but physics settles them at about
# 0.026 m on the table. Use the settled centre height for the first layer.
TABLE_BRICK_CENTER_Z = 0.026

# Extra Z lift above pickup / placement for hover waypoints (metres)
DEFAULT_HOVER_Z: float = 0.12

# Safe home joint configuration -- elevated slightly above table to avoid
# MoveIt goal-state collision rejection at the Gazebo spawn frame.
SAFE_HOME_NAMES: list[str] = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
SAFE_HOME_POSITIONS: list[float] = [1.57, 0.30, -0.18, 0.0, 0.94, -1.57]

_WORLD_NAME = "irb120_workcell"

# Execution modes
MODE_DRY_RUN = "dry_run"  # plan only, no motion
MODE_SIM = "sim"  # plan + execute in Gazebo simulation


# ===========================================================================
# Gazebo helpers (sim mode -- subprocess, no ROS spin)
# ===========================================================================


def _gz_remove(name: str) -> None:
    """Remove a named model from Gazebo."""
    cmd = (
        f"gz service -s /world/{_WORLD_NAME}/remove "
        f"--reqtype gz.msgs.Entity --reptype gz.msgs.Boolean "
        f"--timeout 500 --req 'name: \"{name}\" type: MODEL'"
    )
    subprocess.run(
        cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )


_SUPPLY_GZ_NAME = "supply_brick_gz"  # Gazebo model name for the feeding brick


def _gz_get_pose(name: str) -> np.ndarray | None:
    """Return a model's current Gazebo pose as XYZ + quaternion XYZW."""
    try:
        output = subprocess.check_output(
            f"gz topic -e -n 1 -t /world/{_WORLD_NAME}/pose/info",
            shell=True,
            text=True,
            timeout=5.0,
        )
    except (OSError, subprocess.SubprocessError):
        return None

    start = re.search(rf'pose\s*\{{\s*name:\s*"{re.escape(name)}"', output)
    if not start:
        return None
    next_pose = re.search(r"\npose\s*\{", output[start.end():])
    end = start.end() + next_pose.start() if next_pose else len(output)
    block = output[start.start():end]

    number = r"([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)"
    position_block = re.search(r"position\s*\{(.*?)\}", block, re.DOTALL)
    orientation_block = re.search(r"orientation\s*\{(.*?)\}", block, re.DOTALL)
    if not position_block or not orientation_block:
        return None

    def field(source: str, key: str, default: float) -> float:
        match = re.search(rf"\b{key}:\s*{number}", source)
        return float(match.group(1)) if match else default

    position = position_block.group(1)
    orientation = orientation_block.group(1)
    return np.array(
        [
            field(position, "x", 0.0),
            field(position, "y", 0.0),
            field(position, "z", 0.0),
            field(orientation, "x", 0.0),
            field(orientation, "y", 0.0),
            field(orientation, "z", 0.0),
            field(orientation, "w", 1.0),
        ]
    )


def _gz_fetch_model_names() -> list[str]:
    """Return all top-level model names visible in the Gazebo pose stream."""
    names: list[str] = []
    try:
        output = subprocess.check_output(
            f"gz topic -e -n 1 -t /world/{_WORLD_NAME}/pose/info",
            shell=True,
            text=True,
            timeout=5.0,
        )
        for block in output.split("pose {"):
            m = re.search(r'name:\s+"([^"]+)"', block)
            if not m:
                continue
            name = m.group(1)
            # Skip link/visual sub-entries that live inside models
            if name not in ("brick_link", "visual"):
                names.append(name)
    except Exception:
        pass
    return names


def _gz_clean_scene(node: "PlanAndExecuteClient | None") -> None:
    """
    Fully clean the Gazebo world and MoveIt planning scene before construction.

    Reset the world, preserve the 20 supply-row bricks, remove stale generated
    models, and clear MoveIt's world collision objects.
    """
    print("[gz] Cleaning Gazebo scene (mirroring demo_validation reset_world)...")

    # Step 1: world reset via the node's existing helper (ControlWorld service)
    if node is not None:
        print("[gz]   Step 1: Triggering ControlWorld reset...")
        ok = node.reset_gazebo_simulation()  # calls /reset_simulation service
        if ok:
            print("[gz]   Reset acknowledged. Waiting for world to settle...")
            time.sleep(1.5)
        else:
            print("[gz]   WARNING: ControlWorld reset failed or timed out.")

    # Preserve brick_00..brick_19: these are the physical supply row.
    print("[gz]   Step 2: Preserving supply bricks brick_00..brick_19...")

    print("[gz]   Step 3: Removing stale generated brick models...")
    names = _gz_fetch_model_names()
    to_remove = [
        n for n in names if n.startswith("construct_brick_") or n == _SUPPLY_GZ_NAME
    ]
    for name in to_remove:
        _gz_remove(name)
    if to_remove:
        print(f"[gz]   Removed {len(to_remove)} leftover construct model(s).")

    time.sleep(1.0)  # let Gazebo finish all removals

    stale = [
        name
        for name in _gz_fetch_model_names()
        if name.startswith("construct_brick_") or name == _SUPPLY_GZ_NAME
    ]
    if stale:
        raise RuntimeError("Gazebo cleanup left stale models: " + ", ".join(stale))

    names = set(_gz_fetch_model_names())
    missing = [name for name in SUPPLY_BRICK_NAMES if name not in names]
    if missing:
        raise RuntimeError("Gazebo supply row is incomplete: " + ", ".join(missing))

    # Step 4: clear MoveIt planning scene
    if node is not None:
        print("[gz]   Step 4: Clearing MoveIt planning scene...")
        node.remove_all_world_collision_objects()

    print("[gz] Scene cleanup complete.")


# ===========================================================================
# Helpers
# ===========================================================================


def prepare_goal_poses(demo_poses: list[np.ndarray]) -> list[np.ndarray]:
    """Select 20 goals and align the first sequence layer to the tabletop."""
    selected = [pose.copy() for pose in demo_poses[:MAX_BRICKS]]
    if not selected:
        return selected
    z_offset = TABLE_BRICK_CENTER_Z - min(float(pose[2]) for pose in selected)
    for pose in selected:
        pose[2] += z_offset
    print(
        f"[construct] Using first {len(selected)} goals; "
        f"applied structure Z offset {z_offset:+.6f} m"
    )
    return selected


def _frame_to_pose(frame: dict, index: int) -> np.ndarray:
    """Convert one Rhino/Grasshopper plane (millimetres) to an XYZW pose."""
    try:
        origin = np.asarray(frame["origin"], dtype=float).ravel()
        x_axis = np.asarray(frame["x_axis"], dtype=float).ravel()
        y_axis = np.asarray(frame["y_axis"], dtype=float).ravel()
        z_axis = np.asarray(frame["z_axis"], dtype=float).ravel()
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"Placement frame {index} is missing a valid origin/axis: {exc}") from exc

    if any(value.size != 3 for value in (origin, x_axis, y_axis, z_axis)):
        raise ValueError(f"Placement frame {index} origin and axes must each have 3 values.")
    if not all(np.all(np.isfinite(value)) for value in (origin, x_axis, y_axis, z_axis)):
        raise ValueError(f"Placement frame {index} contains a non-finite value.")

    rotation = np.column_stack((x_axis, y_axis, z_axis))
    gram = rotation.T @ rotation
    if not np.allclose(gram, np.eye(3), atol=1e-5) or not np.isclose(
        np.linalg.det(rotation), 1.0, atol=1e-5
    ):
        raise ValueError(
            f"Placement frame {index} axes must form an orthonormal, right-handed frame."
        )

    return np.concatenate((origin / 1000.0, R.from_matrix(rotation).as_quat()))


def load_poses_file(poses_path: str) -> list[np.ndarray]:
    """
    Load poses from the standalone JSON input format.

    Euler XYZ angles are converted to the XYZW quaternion expected by the
    planner. The file must declare its frame, position units, and orientation
    representation so Grasshopper exports cannot be interpreted ambiguously.
    """
    with open(poses_path, encoding="utf-8") as f:
        data = json.load(f)

    if not isinstance(data, dict):
        raise ValueError("Pose input must be a JSON object.")

    # Native Rhino/Grasshopper export. Origins are brick centres in mm and
    # axis vectors are the columns of the brick-to-world rotation matrix.
    frames = data.get("offset_placement_frames")
    if frames is not None:
        if not isinstance(frames, list) or not frames:
            raise ValueError('"offset_placement_frames" must be a non-empty JSON array.')
        poses = []
        for index, frame in enumerate(frames):
            if not isinstance(frame, dict):
                raise ValueError(f"Placement frame {index} must be a JSON object.")
            poses.append(_frame_to_pose(frame, index))
        print(f"[construct] Loaded {len(poses)} Rhino placement frames from {poses_path}")
        return poses

    # Legacy explicit pose format retained for small hand-written test files.
    if data.get("frame") != "world":
        raise ValueError("Pose input must declare \"frame\": \"world\".")
    position_units = data.get("position_units")
    if position_units not in ("m", "mm"):
        raise ValueError("Pose input must declare \"position_units\": \"m\" or \"mm\".")

    orientation_format = data.get("orientation_format")
    angle_units = data.get("angle_units")
    if orientation_format not in ("euler_xyz", "quaternion_xyzw"):
        raise ValueError(
            "\"orientation_format\" must be \"euler_xyz\" or \"quaternion_xyzw\"."
        )
    if orientation_format == "euler_xyz" and angle_units not in ("degrees", "radians"):
        raise ValueError(
            "Euler input must declare \"angle_units\": \"degrees\" or \"radians\"."
        )

    bricks = data.get("bricks")
    if not isinstance(bricks, list) or not bricks:
        raise ValueError("\"bricks\" must be a non-empty JSON array.")

    poses: list[np.ndarray] = []
    for index, brick in enumerate(bricks):
        if not isinstance(brick, dict):
            raise ValueError(f"Brick {index} must be a JSON object.")
        position = np.asarray(brick.get("position"), dtype=float).ravel()
        orientation = np.asarray(brick.get("orientation"), dtype=float).ravel()
        expected_size = 3 if orientation_format == "euler_xyz" else 4
        if position.size != 3 or orientation.size != expected_size:
            raise ValueError(
                f"Brick {index} needs a 3-value position and "
                f"{expected_size}-value {orientation_format} orientation."
            )
        if not np.all(np.isfinite(position)) or not np.all(np.isfinite(orientation)):
            raise ValueError(f"Brick {index} contains a non-finite pose value.")
        if position_units == "mm":
            position = position / 1000.0

        if orientation_format == "euler_xyz":
            quaternion_xyzw = R.from_euler(
                "xyz", orientation, degrees=(angle_units == "degrees")
            ).as_quat()
        else:
            if np.linalg.norm(orientation) == 0.0:
                raise ValueError(f"Brick {index} has a zero-length quaternion.")
            quaternion_xyzw = R.from_quat(orientation).as_quat()

        poses.append(np.concatenate((position, quaternion_xyzw)))

    print(f"[construct] Loaded {len(poses)} bricks from {poses_path}")
    return poses


def apply_grasp_offset(
    brick_pose_7d: np.ndarray,
    T_grasp_in_brick: np.ndarray,
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """
    Compute the gripper TCP pose in the world frame.

    T_tcp_world = T_brick_world @ T_grasp_in_brick

    Returns
    -------
    tcp_xyz        : (x, y, z) position in world frame
    tcp_quat_xyzw  : (qx, qy, qz, qw) orientation in world frame
    """
    brick = Brick(pose_7d=brick_pose_7d)
    T_brick_world = brick.to_homogeneous_matrix()
    T_tcp_world = T_brick_world @ T_grasp_in_brick

    tcp_xyz = tuple(float(v) for v in T_tcp_world[:3, 3])
    tcp_quat_xyzw = tuple(
        float(v) for v in R.from_matrix(T_tcp_world[:3, :3]).as_quat()
    )
    return tcp_xyz, tcp_quat_xyzw  # type: ignore[return-value]


def hover_above(
    xyz: tuple[float, float, float],
    hover_z: float,
) -> tuple[float, float, float]:
    """Return a pose directly above xyz by hover_z metres."""
    return (xyz[0], xyz[1], xyz[2] + hover_z)


# ===========================================================================
# Per-brick pick-and-place planner
# ===========================================================================


def _extract_last_state(traj) -> tuple[list[str], list[float]] | None:
    if traj is None:
        return None
    jt = traj.joint_trajectory
    if not jt.points:
        return None
    return list(jt.joint_names), list(jt.points[-1].positions)


def plan_brick_sequence(
    node: "PlanAndExecuteClient | None",
    *,
    supply_7d: np.ndarray,
    goal_7d: np.ndarray,
    T_rel: np.ndarray,
    hover_z: float,
) -> Optional[list[object]]:
    """
    Try to plan all phases (Hover Supply -> Grasp Supply -> ... -> Retract Goal).
    Chains the trajectory end-state as the start-state of the next phase.
    Returns the list of 6 planned trajectories if ALL succeed, else None.
    """
    log_info = node.get_logger().info if node else print

    # -- Compute goal TCP pose (apply grasp offset to goal brick frame) ------
    goal_tcp_xyz, goal_tcp_quat = apply_grasp_offset(goal_7d, T_rel)

    # -- Compute supply TCP pose (apply SAME grasp offset to supply brick frame)
    supply_tcp_xyz, supply_tcp_quat = apply_grasp_offset(supply_7d, T_rel)

    supply_hover = hover_above(supply_tcp_xyz, hover_z)
    goal_hover = hover_above(goal_tcp_xyz, hover_z)

    if node is None:
        # Offline mode
        return [None] * 7  # type: ignore

    # ------------------------------------------------------------------
    # Fast IK pre-check: reject geometrically infeasible poses in ~10 ms
    # before spending up to 6 s on OMPL per phase.
    # ------------------------------------------------------------------
    if not node.check_ik(
        "arm",
        "gripper_tcp",
        "world",
        supply_tcp_xyz,
        supply_tcp_quat,
        start_joint_names=SAFE_HOME_NAMES,
        start_joint_positions=SAFE_HOME_POSITIONS,
    ):
        log_info("    [IK SKIP] Supply TCP infeasible for top-down grasp")
        return None
    if not node.check_ik(
        "arm",
        "gripper_tcp",
        "world",
        goal_tcp_xyz,
        goal_tcp_quat,
        start_joint_names=SAFE_HOME_NAMES,
        start_joint_positions=SAFE_HOME_POSITIONS,
    ):
        log_info("    [IK SKIP] Goal TCP infeasible for top-down grasp")
        return None

    plans = []
    current_start_state = None

    def _plan_phase(
        label: str, xyz, quat, lock_wrist: bool = False, wrist_tolerance: float = 0.4
    ) -> bool:
        nonlocal current_start_state
        start_names, start_positions = (
            current_start_state if current_start_state else (None, None)
        )

        traj = node.plan_arm_to_pose_constraints(
            group_name="arm",
            link_name="gripper_tcp",
            frame_id="world",
            goal_xyz=tuple(float(v) for v in xyz),
            goal_quat_xyzw=tuple(float(v) for v in quat),
            joint_4_constraints=3.14,  # ±π keeps IK workspace routing intact
            joint_6_constraints=3.14,
            allowed_planning_time=6.0,
            num_attempts=5,
            start_joint_names=start_names,
            start_joint_positions=start_positions,
            lock_wrist_to_start=lock_wrist,
            lock_wrist_tolerance=wrist_tolerance,
        )
        if traj is None:
            log_info(f"    [FAIL] Plan failed at {label} for top-down grasp")
            return False

        plans.append(traj)
        next_state = _extract_last_state(traj)
        if next_state:
            # Warn if j6 rotated more than ~120° during this phase so we can
            # track wrist-spin without blocking planning.
            if current_start_state:
                prev_names, prev_pos = current_start_state
                if "joint_6" in prev_names:
                    j6_prev = prev_pos[prev_names.index("joint_6")]
                    new_names, new_pos = next_state
                    if "joint_6" in new_names:
                        j6_delta = abs(new_pos[new_names.index("joint_6")] - j6_prev)
                        if j6_delta > 2.09:  # 2.09 rad ≈ 120°
                            log_info(
                                f"    [j6-spin] {label}: j6 rotated "
                                f"{j6_delta:.2f} rad ({np.degrees(j6_delta):.0f}°)"
                            )
            current_start_state = next_state
        return True

    # Offline MoveIt attachment object id
    ghost_id = "ghost_brick"

    if not _plan_phase("hover_supply", supply_hover, supply_tcp_quat):
        return None
    if not _plan_phase(
        "grasp_supply", supply_tcp_xyz, supply_tcp_quat, lock_wrist=True
    ):
        return None

    # GHOST ATTACH: For the transit phases, tell MoveIt the robot is holding the brick
    if node:
        node.attach_box_to_gripper(ghost_id, BRICK_SIZE_XYZ)

    try:
        if not _plan_phase(
            "lift_supply", supply_hover, supply_tcp_quat, lock_wrist=True
        ):
            return None
        # hover_goal is the long transit: do NOT lock the wrist here.
        # Locking j6 to the lift-supply end value causes OMPL to fail when
        # the natural IK solution requires a wrist rotation > the tolerance
        # (observed: ~3.1 rad change in j6 between lift and hover_goal).
        # The spin is cosmetic; correctness requires leaving the wrist free.
        if not _plan_phase("hover_goal", goal_hover, goal_tcp_quat):
            return None
        if not _plan_phase("place_goal", goal_tcp_xyz, goal_tcp_quat, lock_wrist=True):
            return None
    finally:
        # GHOST DETACH: Release the attached object.  MoveIt puts detached objects
        # back into the world scene at the gripper TCP position -- explicitly remove
        # it so it doesn't block subsequent bricks' path planning.
        if node:
            node.detach_box_from_gripper(ghost_id)
            node.remove_scene_object(ghost_id)

    if not _plan_phase("retract_goal", goal_hover, goal_tcp_quat, lock_wrist=True):
        return None

    def _plan_return_home() -> bool:
        nonlocal current_start_state

        start_names, start_positions = (
            current_start_state if current_start_state else (None, None)
        )
        t_names = SAFE_HOME_NAMES
        t_positions = SAFE_HOME_POSITIONS

        traj = node.plan_gripper_to_joint_positions(
            group_name="arm",
            goal_joint_names=t_names,
            goal_joint_positions=t_positions,
            start_joint_names=start_names,
            start_joint_positions=start_positions,
            tolerance=0.08,
            allowed_planning_time=6.0,
            num_attempts=5,
        )
        if traj is None:
            log_info("    [FAIL] Plan failed at return_home for top-down grasp")
            return False

        plans.append(traj)
        next_state = _extract_last_state(traj)
        if next_state:
            current_start_state = next_state
        return True

    if not _plan_return_home():
        return None

    return plans


def execute_brick_sequence(
    node: "PlanAndExecuteClient | None",
    plans: list[object],
    mode: str,
) -> bool:
    """
    Execute the pre-planned trajectories.
    Handles the gripper commands for one existing Gazebo supply brick.
    """
    dry_run = mode == MODE_DRY_RUN
    if node is None:
        return True

    def _exec(label: str, traj) -> bool:
        if dry_run:
            return True
        return node.execute_moveit_trajectory(traj)

    # 0. Open gripper
    if not dry_run:
        node.send_gripper_command(position=0.0, max_velocity=0.05)

    # 1. Hover above supply
    if not _exec("hover_supply", plans[0]):
        return False

    # 2. Move to grasp supply
    if not _exec("grasp_supply", plans[1]):
        return False

    # 3. Close gripper
    if not dry_run:
        node.send_gripper_command(position=0.01, max_velocity=0.05)
    if not dry_run:
        time.sleep(0.5)

    # 4. Lift out from supply
    if not _exec("lift_supply", plans[2]):
        return False

    # 5. Hover above goal
    if not _exec("hover_goal", plans[3]):
        return False

    # 6. Move to goal grasp pose
    if not _exec("place_goal", plans[4]):
        return False

    # 7. Open gripper
    if not dry_run:
        node.send_gripper_command(position=0.0, max_velocity=0.05)
    if not dry_run:
        time.sleep(0.5)

    # 8. Retract above goal
    if not _exec("retract_goal", plans[5]):
        return False

    # 9. Return to home state
    if len(plans) > 6 and plans[6] is not None:
        if not _exec("return_home", plans[6]):
            return False

    return True


# ===========================================================================
# Main construction loop
# ===========================================================================


def run_construction(
    node: Optional["PlanAndExecuteClient"],
    demo_poses: list[np.ndarray],
    *,
    hover_z: float,
    mode: str,
) -> None:
    """
    Iterate through the demo sequence and pick-and-place each brick.

    For each brick:
    - Pick brick_00 through brick_19 in numeric order.
    - Plan every brick with the fixed orthogonal top-down grasp.
    - Register the successfully placed brick as a MoveIt collision object.
    - Stop immediately if a brick pose cannot be queried, planned, or executed.
    """

    goal_poses = prepare_goal_poses(demo_poses)
    placed_count = 0
    failed_bricks: list[int] = []

    print(f"\n[construct] Starting construction -- {len(goal_poses)} bricks")
    mode_label = {
        MODE_DRY_RUN: "DRY-RUN (planning only, no execution)",
        MODE_SIM: "SIM    (plan + execute in Gazebo simulation)",
    }.get(mode, mode)
    print(f"[construct] Mode: {mode_label}\n")

    # In sim mode, perform a full scene cleanup (mirrors demo_validation reset_world)
    # before starting the construction sequence.
    if mode == MODE_SIM:
        _gz_clean_scene(node)

    # Pre-publish the table surface as a collision box so MoveIt avoids it.
    if node is not None:
        node.publish_scene_box(
            object_id="table_surface",
            frame_id="world",
            size_xyz=(2.0, 2.0, 0.02),
            position_xyz=(0.0, 0.0, -0.02),
        )

    # Ensure the robot is physically at the exact SAFE_HOME_POSITIONS before any plans run
    if node is not None and mode == MODE_SIM:
        print("[construct] Physically aligning to SAFE_HOME to initialize geometric baseline...")
        init_traj = node.plan_gripper_to_joint_positions(
            group_name="arm",
            goal_joint_names=SAFE_HOME_NAMES,
            goal_joint_positions=SAFE_HOME_POSITIONS,
            start_joint_names=None,
            start_joint_positions=None,
            tolerance=0.08,
            allowed_planning_time=10.0,
            num_attempts=5,
        )
        if init_traj:
            node.execute_moveit_trajectory(init_traj)
            node.send_gripper_command(position=0.0, max_velocity=0.05)
        else:
            print("[WARN] Initialization move to SAFE_HOME failed. First trajectory might reject!")

    for brick_idx, goal_7d in enumerate(goal_poses):
        supply_name = SUPPLY_BRICK_NAMES[brick_idx]
        print(
            f"\n[construct] -- Brick {brick_idx + 1}/{len(goal_poses)} "
            f"({supply_name}) --"
        )

        print("  [info] Using orthogonal top-down grasp")
        supply_7d = _gz_get_pose(supply_name) if mode == MODE_SIM else None
        if supply_7d is None:
            if mode == MODE_DRY_RUN:
                print(f"  [dry] Gazebo pose unavailable; skipping planning for {supply_name}")
                placed_count += 1
                continue
            print(f"  [FAIL] Could not query Gazebo pose for {supply_name}. Terminating.")
            failed_bricks.append(brick_idx)
            break

        print(
            "  [gz] Supply pose: "
            f"({supply_7d[0]:.3f}, {supply_7d[1]:.3f}, {supply_7d[2]:.3f})"
        )
        plans = plan_brick_sequence(
            node,
            supply_7d=supply_7d,
            goal_7d=goal_7d,
            T_rel=T_GRASP_IN_BRICK,
            hover_z=hover_z,
        )
        if plans is None:
            print(f"  [FAIL] No valid top-down plan for {supply_name}. Terminating.")
            failed_bricks.append(brick_idx)
            break

        exec_ok = execute_brick_sequence(node, plans, mode)

        if not exec_ok:
            print("  [FAIL] Plan executed poorly, terminating early.")
            failed_bricks.append(brick_idx)
            break

        placed_count += 1

        # Register the placed brick as a permanent MoveIt collision object.
        if node is not None:
            brick_obj_id = f"placed_brick_{brick_idx:03d}"
            brick_quat_xyzw = tuple(float(v) for v in goal_7d[3:])
            node.publish_scene_box(
                object_id=brick_obj_id,
                frame_id="world",
                size_xyz=BRICK_SIZE_XYZ,
                position_xyz=tuple(float(v) for v in goal_7d[:3]),
                quat_xyzw=brick_quat_xyzw,
            )
            print(f"  [scene] Added collision object '{brick_obj_id}'")

        # Optional: rest slightly
        if not (mode == MODE_DRY_RUN):
            time.sleep(1.0)

    # -- Summary -----------------------------------------------------------
    print(f"\n{'=' * 60}")
    print("[construct] Construction complete.")
    print(f"  Placed  : {placed_count}/{len(goal_poses)} bricks")
    print(f"  Skipped : {len(failed_bricks)} bricks (indices: {failed_bricks})")
    print(f"{'=' * 60}\n")


# ===========================================================================
# CLI
# ===========================================================================


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=(
            "Simulation pick-and-place construction from a world-frame pose file.\n"
            "Modes:\n"
            "  (default)  dry-run: plan trajectories, report pass/fail, no motion\n"
            "  --sim     : plan + execute in Gazebo simulation"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    p.add_argument(
        "--sim",
        action="store_true",
        default=False,
        help="Pick the 20 existing Gazebo row bricks and execute their placements",
    )
    p.add_argument(
        "--poses-file",
        default=None,
        metavar="JSON_FILE",
        help="World-frame brick pose JSON. See inputs/poses.example.json.",
    )
    p.add_argument(
        "--hover-z",
        type=float,
        default=DEFAULT_HOVER_Z,
        metavar="METRES",
        help=f"Hover height above pick/place poses in metres (default: {DEFAULT_HOVER_Z})",
    )
    return p.parse_args()


def main() -> None:
    args = parse_args()

    if not args.poses_file:
        raise SystemExit("--poses-file is required.")
    demo_poses = load_poses_file(args.poses_file)

    # -- Determine execution mode -------------------------------------------
    if args.sim:
        mode = MODE_SIM
    else:
        mode = MODE_DRY_RUN

    # -- ROS 2 / MoveIt setup -----------------------------------------------
    node = None
    if HAVE_ROS2:
        rclpy.init()
        node = PlanAndExecuteClient()
        print("[construct] PlanAndExecuteClient ready.")
        print(f"[construct] Mode: {mode}")
    else:
        print(
            "[construct] ROS 2 not available -- running in offline/print-only mode.\n"
            "            Install rclpy and moveit_client dependencies "
            "for actual planning."
        )
        mode = MODE_DRY_RUN  # force dry-run if no ROS

    # In non-sim modes, clear any leftover MoveIt collision objects.
    # (In sim mode _gz_clean_scene() handles this inside run_construction.)
    if node is not None and mode != MODE_SIM:
        node.remove_all_world_collision_objects()

    try:
        run_construction(
            node,
            demo_poses,
            hover_z=args.hover_z,
            mode=mode,
        )
    finally:
        if node is not None:
            # Spin briefly to flush any pending DDS messages
            for _ in range(60):
                rclpy.spin_once(node, timeout_sec=0.05)
            node.destroy_node()
            rclpy.shutdown()
            print("[construct] ROS 2 shutdown complete.")


if __name__ == "__main__":
    main()
