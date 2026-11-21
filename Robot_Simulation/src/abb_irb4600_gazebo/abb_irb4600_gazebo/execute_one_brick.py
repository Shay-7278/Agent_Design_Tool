"""LL assembly execution; one-brick and full-wall entry points share this code."""
from __future__ import annotations

import argparse
import json
import time
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import TransformStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock

from .brick_lifecycle import BrickLifecycle, BrickPlan, BrickState, load_pickup_layout, load_placements
from .gz_brick_world import GazeboBrickWorld
from .brick_models import load_brick_models, pickup_pose_on_platform
from .irb4600_kinematics import solve_base_target
from .motion import BrickMotionTargets, MotionKind, Pose, build_pick_place_steps, rail_position
from .target_motion import brick_frame_from_tcp, flange_target_from_tcp, quaternion_slerp, rpy_to_quaternion, tcp_target_from_brick

ARM_JOINTS = ["joint_LL_ext_1", *[f"joint_LL_{i}" for i in range(1, 7)]]
GRIPPER_JOINTS = ["LL_schunk_left_finger_joint", "LL_schunk_right_finger_joint"]


def _positive_speed_scale(value):
    try:
        scale = float(value)
    except (ValueError, TypeError) as exc:
        raise argparse.ArgumentTypeError('speed scale must be a finite positive number') from exc
    if not math.isfinite(scale) or scale <= 0:
        raise argparse.ArgumentTypeError('speed scale must be a finite positive number')
    return scale


class TrajectoryWatchdog:
    """Allow slow Gazebo motion while bounding stopped clocks and missing results."""
    def __init__(self, sim_now, wall_now, duration):
        self.start_sim = self.last_sim = sim_now
        self.last_clock_progress_wall = wall_now
        self.sim_budget = duration + 10.0

    def check(self, sim_now, wall_now):
        if sim_now < self.last_sim - 0.001:
            raise RuntimeError("Gazebo clock reset during trajectory")
        if sim_now > self.last_sim + 1e-9:
            self.last_sim = sim_now
            self.last_clock_progress_wall = wall_now
        if wall_now - self.last_clock_progress_wall > 30.0:
            raise RuntimeError("Gazebo clock stopped or simulation paused for 30 real seconds")
        if sim_now - self.start_sim > self.sim_budget:
            raise RuntimeError("controller returned no result within trajectory duration + 10 simulation seconds")


def _qmul(a, b):
    ax, ay, az, aw = a; bx, by, bz, bw = b
    return (aw*bx+ax*bw+ay*bz-az*by, aw*by-ax*bz+ay*bw+az*bx,
            aw*bz+ax*by-ay*bx+az*bw, aw*bw-ax*bx-ay*by-az*bz)


def _qrot(q, v):
    return _qmul(_qmul(q, (*v, 0.0)), (-q[0], -q[1], -q[2], q[3]))[:3]


def _transform_pose(pose: Pose, tf: TransformStamped) -> Pose:
    t = tf.transform.translation
    q = (tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w)
    rotated = _qrot(q, pose.position)
    return Pose((rotated[0]+t.x, rotated[1]+t.y, rotated[2]+t.z), _qmul(q, pose.orientation_xyzw))


def _compose_pose(parent: Pose, child: Pose) -> Pose:
    rotated = _qrot(parent.orientation_xyzw, child.position)
    return Pose(tuple(a+b for a,b in zip(parent.position, rotated)),
                _qmul(parent.orientation_xyzw, child.orientation_xyzw))


class OneBrickExecutor(Node):
    def __init__(self, placement_file: Path, pickup_file: Path, sdf_path: Path,
                 count: int | None = 1, placement_frame: str = "MT_base",
                 preflight_only: bool = False,
                 observations: Path = Path('/ros2_ws/outputs/assembly_observations.jsonl'),
                 start_brick: int = 1, speed_scale: float = 1.0) -> None:
        self.speed_scale = _positive_speed_scale(speed_scale)
        super().__init__("execute_one_brick")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.arm = ActionClient(self, FollowJointTrajectory, "/LL_manipulator_controller/follow_joint_trajectory")
        self.gripper = ActionClient(self, FollowJointTrajectory, "/LL_gripper_controller/follow_joint_trajectory")
        self.placements = load_placements(placement_file)
        self.placement_file = Path(placement_file)
        self._release_observations = {}
        if not self.placements:
            raise ValueError("placement file is empty")
        self.input_count = len(self.placements)
        model_dir = Path(get_package_share_directory('abb_irb4600_gazebo')) / 'models'
        models = load_brick_models(placement_file, model_dir)
        self.half_rows = tuple(i+1 for i,p in enumerate(models) if p.parent.name == 'brick_half')
        if count is not None:
            self.placements = self.placements[:count]
        if not 1 <= start_brick <= len(self.placements):
            raise ValueError("--start-brick must be within the selected placement rows")
        self.start_index = start_brick - 1
        # The CAD export has an obsolete centre Z. Both selected full/half
        # models are 57 mm high; match the platform/model-based spawn rule.
        self.pickups = tuple(pickup_pose_on_platform(p, model_dir / 'brick/model.sdf')
                             for p in load_pickup_layout(pickup_file))
        self.placement_frame = placement_frame
        self.preflight_only = preflight_only
        self.observations = observations
        self.current_plan = None
        profile_path = Path(get_package_share_directory('abb_irb4600_gazebo')) / 'config/gh_motion_profile.json'
        self.profile = json.loads(profile_path.read_text(encoding='utf-8'))
        h = self.profile['home_tcp']
        self.home = brick_frame_from_tcp(Pose(tuple(h['position']), tuple(h['orientation_xyzw'])))
        self.joints = {}
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        self.sim_time = None
        self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.world = GazeboBrickWorld(
            "irb4600_workcell", sdf_path,
            brick_models={f'pickup_brick_{i:03d}': p for i,p in enumerate(models)
                          if p.parent.name == 'brick_half'}, static_supply=True)
        tools_path = Path(get_package_share_directory("abb_irb4600_workcell_description")) / "config" / "tools.yaml"
        with tools_path.open(encoding="utf-8") as stream:
            self.tool_cfg = yaml.safe_load(stream)["tools"]["schunk_pgf_100"]
            tcp = self.tool_cfg["flange_tcp"]
        self.flange_tcp = Pose(tuple(tcp["xyz"]), rpy_to_quaternion(*tcp["rpy"]))
        urdf = ET.fromstring(xacro.process_file(str(tools_path.parent.parent / "urdf" / "abb_irb4600_assembly.xacro")).toxml())
        self.limits = [tuple(float(urdf.find(f"./joint[@name='joint_LL_{i}']/limit").get(k))
                             for k in ("lower", "upper")) for i in range(1, 7)]
        self.velocity_limits = [float(urdf.find(f"./joint[@name='{j}']/limit").get('velocity')) for j in ARM_JOINTS]
        self.base_tf = None

    def _on_joints(self, message):
        self.joints.update(zip(message.name, message.position))

    def _on_clock(self, message):
        self.sim_time = message.clock.sec + message.clock.nanosec * 1e-9

    def _simulation_time(self):
        # Gazebo callbacks run independently while blocking entity RPCs execute.
        # The ROS executor's cached /clock can lag by an entire large prefill;
        # using it as a new action's start would immediately expire its budget.
        native = getattr(getattr(self, 'world', None), '_sim_time', None)
        return native if isinstance(native, (int, float)) else self.sim_time

    def _on_trajectory_feedback(self, message):
        self._feedback_wall = time.monotonic()
        self._feedback_sim = self._simulation_time()

    def _cancel_trajectory(self, handle, result_future):
        cancellation = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancellation, timeout_sec=10.0)
        acknowledged = cancellation.done() and cancellation.result() is not None and bool(cancellation.result().goals_canceling)
        if acknowledged:
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)
        if acknowledged and result_future.done() and result_future.result() is not None:
            return "goal cancellation confirmed"
        return "cancellation NOT confirmed; stop the simulation before retrying"

    def _record_placement(self, name, target):
        # Observation is not a structural acceptance gate or a stability claim.
        record = {'brick': name, 'state': 'PLACED', 'mode': 'assembly_visualization',
                  'target_position': target.position,
                  'target_orientation_xyzw': target.orientation_xyzw}
        record['release_verification'] = getattr(self.world, 'attachment_checks', {}).get(name)
        saved = getattr(self, '_release_observations', {}).pop(name, None)
        try:
            if saved is None:
                raise RuntimeError('no physical post-release observation captured')
            if isinstance(saved, str):
                raise RuntimeError(saved)
            actual, sequence, stamp = saved
            record.update(actual_position=actual.position, actual_orientation_xyzw=actual.orientation_xyzw,
                          pose_sequence=sequence, simulation_stamp=stamp,
                          displacement_m=math.dist(actual.position, target.position),
                          observation_source='physical post-release, before static consolidation')
        except RuntimeError as exc:
            record['observation_error'] = str(exc)
        self.observations.parent.mkdir(parents=True, exist_ok=True)
        with self.observations.open('a', encoding='utf-8') as stream:
            stream.write(json.dumps(record) + '\n')
        self.get_logger().info(f"{name} PLACED (command complete); observation={record}")

    def _capture_release_observation(self, name):
        if not hasattr(self, '_release_observations'):
            self._release_observations = {}
        try:
            self._release_observations[name] = self.world.observation(name)
        except (RuntimeError, AttributeError) as exc:
            self._release_observations[name] = str(exc)

    def _export_assembly_manifest(self):
        bricks = self.world.static_wall_manifest()
        selected_count = len(getattr(self, 'placements', bricks))
        manifest = {'frame': 'workcell_base', 'mode': 'assembly_visualization',
                    'placement_file': str(getattr(self, 'placement_file', '')),
                    'placement_frame': self.placement_frame, 'bricks': bricks,
                    'input_bricks': getattr(self, 'input_count', selected_count),
                    'selected_bricks': selected_count,
                    'geometry_complete': len(bricks) == selected_count}
        path = self.observations.with_name('assembly_manifest.json')
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')

    def _spin_for(self, seconds):
        if seconds <= 0:
            rclpy.spin_once(self, timeout_sec=0.0)
            return
        deadline = time.monotonic() + 30.0
        while self._simulation_time() is None:
            if time.monotonic() > deadline:
                raise RuntimeError('Gazebo /clock unavailable during GH WaitTime')
            rclpy.spin_once(self, timeout_sec=0.1)
        start = self._simulation_time()
        watchdog = TrajectoryWatchdog(start, time.monotonic(), seconds)
        while self._simulation_time() - start < seconds:
            rclpy.spin_once(self, timeout_sec=0.1)
            watchdog.check(self._simulation_time(), time.monotonic())

    def send(self, client, names, values, seconds=3.0, samples=None):
        clock_deadline = time.monotonic() + 20.0
        while self._simulation_time() is None:
            if time.monotonic() > clock_deadline:
                raise RuntimeError("Gazebo /clock unavailable; trajectory was not sent")
            rclpy.spin_once(self, timeout_sec=0.1)
        if not client.wait_for_server(timeout_sec=20.0):
            raise RuntimeError(f"action server unavailable: {client._action_name}")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = names
        point = JointTrajectoryPoint(); point.positions = list(values)
        point.velocities = [0.0] * len(values)
        point.accelerations = [0.0] * len(values)
        ns = round(seconds * 1e9)
        point.time_from_start = Duration(sec=ns//1000000000, nanosec=ns%1000000000)
        goal.trajectory.points = samples or [point]
        self._feedback_wall = time.monotonic()
        self._feedback_sim = self._simulation_time()
        future = client.send_goal_async(goal, feedback_callback=self._on_trajectory_feedback)
        rclpy.spin_until_future_complete(self, future, timeout_sec=20.0)
        if not future.done():
            raise RuntimeError("trajectory acceptance timed out; stop simulation before retrying because acceptance is unknown")
        handle = future.result()
        if handle is None or not handle.accepted: raise RuntimeError("trajectory rejected")
        result_future = handle.get_result_async()
        duration = goal.trajectory.points[-1].time_from_start
        start_wall = last_log = time.monotonic()
        watchdog = TrajectoryWatchdog(self._simulation_time(), start_wall, duration.sec + duration.nanosec * 1e-9)
        try:
            while rclpy.ok() and not result_future.done():
                rclpy.spin_once(self, timeout_sec=0.1)
                if result_future.done():
                    break
                now = time.monotonic()
                watchdog.check(self._simulation_time(), now)
                # Both clocks must advance enough before declaring feedback lost;
                # very slow Gazebo alone must not trip a real-time deadline.
                if now - self._feedback_wall > 15 and self._simulation_time() - self._feedback_sim > 2:
                    raise RuntimeError("controller feedback stopped while Gazebo clock continued")
                if now - last_log >= 15:
                    elapsed = self._simulation_time() - watchdog.start_sim
                    self.get_logger().info(
                        f"trajectory in progress: {elapsed:.2f}/{duration.sec + duration.nanosec*1e-9:.2f} sim s; "
                        f"{now-start_wall:.0f} real s; simulation speed={elapsed/max(now-start_wall,0.001):.3f}x")
                    last_log = now
            if not result_future.done() or result_future.result() is None:
                raise RuntimeError("trajectory ended without a controller result")
        except (RuntimeError, KeyboardInterrupt) as exc:
            cancellation_status = self._cancel_trajectory(handle, result_future)
            raise RuntimeError(f"{exc}; {cancellation_status}") from exc
        result = result_future.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(f"trajectory failed: {result.error_string}")

    def run(self):
        if hasattr(self, 'half_rows'):
            selected = [i for i in self.half_rows if i <= len(self.placements)]
            self.get_logger().info(
                f'Brick sizes: {len(self.placements)-len(selected)} full, {len(selected)} half; '
                f'half-brick source rows: {selected}')
        self.get_logger().info(
            f'Motion speed scale: {getattr(self, "speed_scale", 1.0):g}x; '
            'URDF velocity limits remain active; GH waits and gripper timing are unchanged')
        deadline = time.monotonic() + 40.0
        while not (all(self.tf_buffer.can_transform(a, b, rclpy.time.Time())
                       for a, b in [("LL_base_link", "workcell_base"),
                                    ("workcell_base", "LL_track"),
                                    ("workcell_base", self.placement_frame)])
                   and all(j in self.joints for j in ARM_JOINTS)):
            if time.monotonic() > deadline:
                raise RuntimeError("required TF or LL joint states unavailable")
            rclpy.spin_once(self, timeout_sec=0.1)
        # Record the reference rail position with this TF; restarts at a moved
        # carriage must use rail delta, not assume the carriage is at zero.
        self.base_tf = self.tf_buffer.lookup_transform(
            "LL_base_link", "workcell_base", rclpy.time.Time()
        )
        self.reference_rail = self.joints[ARM_JOINTS[0]]
        self.last_rail = self.reference_rail
        self._cache_fixed_transforms()
        placement_tf = self.tf_buffer.lookup_transform("workcell_base", self.placement_frame, rclpy.time.Time())
        placements = [_transform_pose(item.pose, placement_tf) for item in self.placements]
        self.seed = tuple(self.joints[j] for j in ARM_JOINTS[1:])
        t = self.tf_buffer.lookup_transform('workcell_base','LL_schunk_tcp',rclpy.time.Time()).transform
        self.last_target = brick_frame_from_tcp(Pose((t.translation.x,t.translation.y,t.translation.z),
                                                    (t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w)))
        failures = []
        planned_start = self.last_target
        planned_seed = self.seed
        planned_rail = self.reference_rail
        start_index = getattr(self, 'start_index', 0)
        for index in range(start_index, len(placements)):
            placement = placements[index]
            targets = BrickMotionTargets(str(index), self.pickups[index % len(self.pickups)], placement)
            for step in build_pick_place_steps(targets, self.profile['retract_m'], self.profile['wait_s'], self.home):
                if step.target is None:
                    continue
                try:
                    points, _, planned_seed = self._plan_motion(planned_start, step.target, step.kind, planned_seed, planned_rail)
                    planned_rail = points[-1].positions[0]
                    planned_start = step.target
                except ValueError as exc:
                    failures.append(f'brick {index+1} {step.kind.value}: {exc}')
        if failures:
            raise RuntimeError(f"preflight failed for {len(failures)} endpoints:\n" + "\n".join(failures[:15]))
        self.get_logger().info(f"GH motion preflight passed: bricks {start_index+1}..{len(placements)} in {self.placement_frame}; home, rail offset, MoveJ and MoveL included")
        if self.preflight_only:
            return
        self._await_startup_batch()
        if start_index:
            self._prefill_placements(placements)
        plans = [BrickPlan(f"pickup_brick_{i:03d}", self.pickups[i % len(self.pickups)], p)
                 for i, p in enumerate(placements) if i >= start_index]
        self.lifecycle = BrickLifecycle(self.world, plans)
        for index in range(start_index, len(placements)):
            placement = placements[index]
            plan = plans[index - start_index]
            self.current_plan = plan
            # Initial startup names are 000..049; every later batch gets new
            # names, so placed bricks remain in the scene and under physics.
            if index > start_index and index % len(self.pickups) == 0:
                for j in range(index, min(index + len(self.pickups), len(placements))):
                    self.world.spawn(f"pickup_brick_{j:03d}", self.pickups[j % len(self.pickups)])
                self._spin_for(2.0)
            name = f"pickup_brick_{index:03d}"
            self.world.activate(name)
            pickup = self.world.pose(name)
            slot = self.pickups[index % len(self.pickups)]
            if math.dist(pickup.position[:2], slot.position[:2]) > 0.005:
                raise RuntimeError(f"{name} is away from its pickup slot; use a fresh world")
            self.get_logger().info(f"brick {index+1}/{len(placements)} AVAILABLE: {name}")
            self.send(self.gripper, GRIPPER_JOINTS, [0.0, 0.0], 2)
            targets = BrickMotionTargets(name, pickup, placement)
            for step in build_pick_place_steps(targets, self.profile['retract_m'], self.profile['wait_s'], self.home):
                if step.kind in (MotionKind.MOVE_J, MotionKind.MOVE_L):
                    self._move_target(step.target, self.flange_tcp, step.kind)
                    state = self.lifecycle.state(plan)
                    if step.kind is MotionKind.MOVE_L and state in (BrickState.ATTACHED, BrickState.TRANSPORTING):
                        carried = self.world.pose(name)
                        orientation_match = abs(sum(a*b for a,b in zip(carried.orientation_xyzw, step.target.orientation_xyzw)))
                        if math.dist(carried.position, step.target.position) > 0.005 or orientation_match < math.cos(math.radians(2)/2):
                            raise RuntimeError(f"{name} did not follow the tool")
                        if state is BrickState.ATTACHED:
                            self.lifecycle.begin_transport(plan)
                            self.get_logger().info(f"{name} TRANSPORTING")
                elif step.kind is MotionKind.WAIT:
                    self._spin_for(step.duration_s)
                elif step.kind is MotionKind.GRIPPER_CLOSE:
                    travel = self.tool_cfg["finger_travel"]
                    self.send(self.gripper, GRIPPER_JOINTS, [travel, travel], 3)
                    self.lifecycle.attach(plan)
                    self.get_logger().info(f"{name} ATTACHED; {getattr(self.world, 'attachment_checks', {}).get(name)}")
                elif step.kind is MotionKind.GRIPPER_OPEN:
                    self.lifecycle.release(plan)
                    self._capture_release_observation(name)
                    self.world.consolidate(name, placement)
                    self._export_assembly_manifest()
                    self.send(self.gripper, GRIPPER_JOINTS, [0.0, 0.0], 3)
                    self.get_logger().info(f"{name} RELEASED and consolidated into static wall; {getattr(self.world, 'attachment_checks', {}).get(name)}")
            # PLACED means GH's placement/release instruction completed.
            # Physics observations are recorded separately, never used to rewrite
            # the JSON or stop the task because a structure moves after release.
            self.lifecycle.mark_placed(plan)
            self._record_placement(name, placement)
        self.get_logger().info(f"ASSEMBLY COMPLETE: {len(plans)} robot-placed bricks; {start_index} PREFILLED")

    def _await_startup_batch(self):
        """Wait for asynchronous startup spawns before validating a fresh scene."""
        count = min(len(self.pickups), getattr(self, 'input_count', len(self.placements)))
        expected = {f'pickup_brick_{i:03d}' for i in range(count)}
        deadline = time.monotonic() + 30.0
        while True:
            models = self.world.models()
            if any(name.startswith('placed_structure_') for name in models):
                raise RuntimeError('a completed wall already exists; use a fresh world')
            missing = expected - set(models)
            if not missing:
                return
            if time.monotonic() >= deadline:
                raise RuntimeError(f'startup pickup batch not ready: {sorted(missing)}; use a fresh world')
            rclpy.spin_once(self, timeout_sec=0.1)

    def _prefill_placements(self, placements):
        """Preplace the skipped prefix in a fresh world; retain absolute IDs/slots."""
        models = self.world.models()
        initial_count = min(len(self.pickups), getattr(self, 'input_count', len(placements)))
        # Validate the entire startup batch before any relocation. Re-running in
        # an already used scene must not teleport an attached or placed brick.
        for index in range(initial_count):
            name = f'pickup_brick_{index:03d}'
            if name not in models:
                raise RuntimeError(f'{name} missing from startup batch; use a fresh world')
            actual = self.world.pose(name)
            if math.dist(actual.position[:2], self.pickups[index].position[:2]) > 0.005:
                raise RuntimeError(f'{name} is away from its pickup slot; use a fresh world')
        if any(name.startswith('pickup_brick_') and name not in
               {f'pickup_brick_{i:03d}' for i in range(initial_count)} for name in models):
            raise RuntimeError('later pickup batches already exist; use a fresh world')
        self.get_logger().info(
            f'PREFILL: placing bricks 1..{self.start_index} directly, then executing brick {self.start_index+1}; '
            'completed placements are static; the current brick is physically simulated.')
        self.world.prefill_static([(f'pickup_brick_{i:03d}', pose)
                                   for i, pose in enumerate(placements[:self.start_index])])
        self._export_assembly_manifest()
        # A start in a later batch also needs that batch's unused pickup slots.
        batch_start = self.start_index // len(self.pickups) * len(self.pickups)
        for index in range(self.start_index, min(batch_start + len(self.pickups), len(placements))):
            name = f'pickup_brick_{index:03d}'
            if name not in models:
                self.world.spawn(name, self.pickups[index % len(self.pickups)])
        self.observations.parent.mkdir(parents=True, exist_ok=True)
        with self.observations.open('a', encoding='utf-8') as stream:
            for index, placement in enumerate(placements[:self.start_index]):
                name = f'pickup_brick_{index:03d}'
                actual, sequence, stamp = self.world.observation(name)
                displacement = math.dist(actual.position, placement.position)
                stream.write(json.dumps({
                    'brick': name, 'state': 'PREFILLED', 'mode': 'assembly_visualization',
                    'observation_source': 'static target geometry', 'target_position': placement.position,
                    'target_orientation_xyzw': placement.orientation_xyzw,
                    'actual_position': actual.position, 'actual_orientation_xyzw': actual.orientation_xyzw,
                    'pose_sequence': sequence, 'simulation_stamp': stamp,
                    'displacement_m': displacement}) + '\n')
                self.get_logger().info(f'{name} PREFILLED; measured displacement={displacement:.6f} m')

    def _cache_fixed_transforms(self):
        """Cache fixed bases and tool mapping; retain rail translation per target."""
        track_tf = self.tf_buffer.lookup_transform("workcell_base", "LL_track", rclpy.time.Time())
        tq = (track_tf.transform.rotation.x, track_tf.transform.rotation.y,
              track_tf.transform.rotation.z, track_tf.transform.rotation.w)
        self.track_origin = (track_tf.transform.translation.x, track_tf.transform.translation.y,
                             track_tf.transform.translation.z)
        self.track_direction = _qrot(tq, (1.0, 0.0, 0.0))
        tf = self.base_tf
        base_q = (tf.transform.rotation.x, tf.transform.rotation.y,
                  tf.transform.rotation.z, tf.transform.rotation.w)
        t = tf.transform.translation
        self.reference_base_pose = Pose((t.x, t.y, t.z), base_q)
        self.rail_direction_base = _qrot(base_q, self.track_direction)
        identity = Pose((0, 0, 0), (0, 0, 0, 1))
        self.brick_flange = flange_target_from_tcp(tcp_target_from_brick(identity), self.flange_tcp)

    def _solve(self, target, seed):
        rail = rail_position(target, self.track_origin, self.track_direction,
                             offset_m=self.profile['track_offset_m'], limits_m=(0.0, 3.89))
        root_target = _compose_pose(self.reference_base_pose, target)
        # The cached base is the pose at reference_rail, not a frozen carriage.
        displacement_local = tuple(v * (self.reference_rail - rail) for v in self.rail_direction_base)
        root_target = Pose(tuple(root_target.position[i] + displacement_local[i] for i in range(3)),
                           root_target.orientation_xyzw)
        flange_target = _compose_pose(root_target, self.brick_flange)
        joints = solve_base_target(flange_target, self.limits, seed, cfx=self.profile['cfx'])
        return rail, joints

    def _plan_motion(self, start, target, kind, seed, start_rail=None):
        end_rail, end_joints = self._solve(target, seed)
        if start_rail is None:
            start_rail = self.last_rail
        scale = getattr(self, 'speed_scale', 1.0)
        speed = self.profile['move_l_speed_m_s' if kind is MotionKind.MOVE_L else 'move_j_speed_m_s'] * scale
        angle = 2 * math.acos(min(1.0, abs(sum(a*b for a,b in zip(start.orientation_xyzw,target.orientation_xyzw)))))
        duration = max(0.5, 1.875*math.dist(start.position,target.position)/speed,
                       1.875*angle/(self.profile['orientation_speed_rad_s'] * scale))
        trajectory = []
        previous = seed
        for i in range(1, 26):
            t = i/25
            u = 10*t**3 - 15*t**4 + 6*t**5
            if kind is MotionKind.MOVE_L:
                pose = Pose(tuple(a+u*(b-a) for a,b in zip(start.position,target.position)),
                            quaternion_slerp(start.orientation_xyzw,target.orientation_xyzw,u))
                rail, q = self._solve(pose, previous)
                if max(abs(a-b) for a,b in zip(q,previous)) > 0.25:
                    raise ValueError('GH MoveL has a joint branch discontinuity')
            else:
                rail = start_rail + u*(end_rail-start_rail)
                q = tuple(a+u*(b-a) for a,b in zip(seed,end_joints))
            trajectory.append([rail,*q])
            previous = q
        first = [start_rail,*seed]
        # Respect URDF actuator speeds even where GH's v_leax exceeds the rail.
        for a,b in zip([first]+trajectory[:-1],trajectory):
            duration = max(duration, max(abs(x-y)*25/limit for x,y,limit in zip(a,b,self.velocity_limits)))
        points = []
        for i, values in enumerate(trajectory, 1):
            p = JointTrajectoryPoint()
            p.positions = list(values)
            ns = round(duration*i/25*1e9)
            p.time_from_start = Duration(sec=ns//1000000000,nanosec=ns%1000000000)
            if i==25:
                p.velocities = [0.0]*7
            points.append(p)
        return points, duration, tuple(trajectory[-1][1:])

    def _wait_for_tcp(self, target: Pose, timeout_sim: float = 10.0) -> None:
        """Wait for fresh measured arrival; controller success alone is insufficient."""
        expected = tcp_target_from_brick(target)
        start_wall = last_progress_wall = time.monotonic()
        start_sim = last_sim = self._simulation_time()
        inside_since = None
        first_stamp = last_tf_stamp = last_tf_progress_sim = None
        feedback_elapsed = 0.0
        samples = []
        last_recorded_stamp = None
        position_error = orientation_error = None
        status = 'TF unavailable'
        outcome = 'FAILED'
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)
                now = time.monotonic()
                sim_now = self._simulation_time()
                if sim_now is not None:
                    if start_sim is None:
                        start_sim = sim_now
                    if last_sim is not None and sim_now < last_sim - 0.001:
                        raise RuntimeError('Gazebo clock reset during tool arrival')
                    if last_sim is None or sim_now > last_sim + 1e-9:
                        last_progress_wall = now
                    last_sim = sim_now
                if now - last_progress_wall > 30.0:
                    raise RuntimeError('Gazebo clock stopped or unavailable for 30 real seconds during tool arrival')
                if start_sim is None:
                    continue
                try:
                    actual_tf = self.tf_buffer.lookup_transform('workcell_base', 'LL_schunk_tcp', rclpy.time.Time())
                    stamp = actual_tf.header.stamp.sec + actual_tf.header.stamp.nanosec * 1e-9
                    t, q = actual_tf.transform.translation, actual_tf.transform.rotation
                    position_error = math.dist((t.x, t.y, t.z), expected.position)
                    match = abs(sum(a*b for a,b in zip((q.x,q.y,q.z,q.w), expected.orientation_xyzw)))
                    orientation_error = math.degrees(2 * math.acos(min(1.0, match)))
                    # TF and native-clock callbacks can have different delivery
                    # delays after a large scene update. Require new TF samples
                    # during this wait instead of comparing their absolute ages.
                    if first_stamp is None:
                        first_stamp = last_tf_stamp = stamp
                        last_tf_progress_sim = sim_now
                    elif stamp > last_tf_stamp + 1e-9:
                        last_tf_stamp = stamp
                        last_tf_progress_sim = sim_now
                        last_progress_wall = now
                    feedback_elapsed = max(0.0, stamp - first_stamp)
                    fresh = (stamp > first_stamp + 1e-9
                             and stamp >= last_tf_stamp - 1e-9
                             and sim_now - last_tf_progress_sim <= 0.25)
                    status = 'fresh TF' if fresh else 'stale TF'
                    inside = fresh and position_error <= 0.003 and orientation_error <= 2.0
                    if not inside:
                        inside_since = None
                    elif inside_since is None:
                        inside_since = stamp
                    arrived = (inside_since is not None
                               and stamp - inside_since >= 0.1 - 1e-9)
                    if last_recorded_stamp is None or stamp - last_recorded_stamp >= 0.1 - 1e-9 or arrived:
                        samples.append({'elapsed_sim_s': sim_now-start_sim, 'elapsed_real_s': now-start_wall,
                                        'tf_stamp': stamp, 'position_error_m': position_error,
                                        'orientation_error_deg': orientation_error, 'fresh': fresh})
                        last_recorded_stamp = stamp
                    if arrived:
                        outcome = 'ARRIVED'
                        self.get_logger().info(
                            f'TCP ARRIVED after {sim_now-start_sim:.3f} sim s: '
                            f'{position_error*1000:.3f} mm, {orientation_error:.3f} deg')
                        return
                except TransformException as exc:
                    status = f'TF unavailable: {exc}'
                    inside_since = None
                if max(sim_now - start_sim, feedback_elapsed) >= timeout_sim:
                    raise RuntimeError(
                        f'tool arrival timed out after {timeout_sim:g} simulation seconds; '
                        f'position_error_m={position_error}, orientation_error_deg={orientation_error}; {status}')
            raise RuntimeError('ROS stopped during tool arrival')
        finally:
            self.observations.parent.mkdir(parents=True, exist_ok=True)
            trace = self.observations.with_name(self.observations.stem + '_tcp_arrival.jsonl')
            with trace.open('a', encoding='utf-8') as stream:
                stream.write(json.dumps({'brick': getattr(getattr(self, 'current_plan', None), 'name', None),
                                         'outcome': outcome, 'target_position': expected.position,
                                         'target_orientation_xyzw': expected.orientation_xyzw,
                                         'timeout_sim_s': timeout_sim, 'status': status,
                                         'samples': samples}) + '\n')

    def _move_target(self, target: Pose, flange_tcp: Pose, kind: MotionKind) -> None:
        self.get_logger().info(f"executing {kind.value}: {target.position}")
        samples, duration, joints = self._plan_motion(self.last_target, target, kind, self.seed)
        if math.dist(self.last_target.position,target.position) < 1e-7 and abs(sum(a*b for a,b in zip(self.last_target.orientation_xyzw,target.orientation_xyzw))) > 0.999999:
            return
        self.send(self.arm, ARM_JOINTS, samples[-1].positions, duration, samples=samples)
        self._wait_for_tcp(target)
        self.seed = joints
        self.last_rail = samples[-1].positions[0]
        self.last_target = target


def main(wall=False) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--placement-file", default="/ros2_ws/inputs/brick_frames_numeric.json")
    description = Path(get_package_share_directory("abb_irb4600_workcell_description"))
    gazebo = Path(get_package_share_directory("abb_irb4600_gazebo"))
    parser.add_argument("--pickup-file", default=str(description / "config/fixed_pickup_layout.json"))
    parser.add_argument("--sdf", default=str(gazebo / "models/brick/model.sdf"))
    parser.add_argument("--placement-frame", default="MT_base")
    parser.add_argument("--preflight-only", action="store_true")
    parser.add_argument("--max-bricks", type=int, default=None if wall else 1)
    parser.add_argument("--start-brick", type=int, default=1,
                        help="1-based first row to execute; earlier rows are preplaced in a fresh world")
    parser.add_argument('--speed-scale', type=_positive_speed_scale, default=1.0,
                        help='Scale arm/rail motion speeds; actuator limits still apply (default: 1)')
    parser.add_argument('--observations',type=Path,default=Path('/ros2_ws/outputs/assembly_observations.jsonl'))
    args, _ = parser.parse_known_args()
    if args.max_bricks is not None and args.max_bricks < 1:
        parser.error("--max-bricks must be positive")
    if args.start_brick < 1:
        parser.error("--start-brick must be positive")
    rclpy.init(); node = OneBrickExecutor(Path(args.placement_file), Path(args.pickup_file), Path(args.sdf),
                                       args.max_bricks, args.placement_frame, args.preflight_only, args.observations,
                                       args.start_brick, args.speed_scale)
    try: node.run()
    except Exception:
        if node.current_plan is not None and hasattr(node,'lifecycle'):
            node.lifecycle.fail(node.current_plan)
            node.get_logger().error(f'{node.current_plan.name} FAILED')
        raise
    finally:
        node.world.close()
        node.destroy_node()
        rclpy.shutdown()


def wall_main():
    main(wall=True)


if __name__ == "__main__": main()
