"""Simulator-neutral motion contracts recovered from the Grasshopper graph.

The target-to-flange conversion and IK boundary live in ``target_motion`` so
the task sequence remains independent of the robot solver.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import isfinite, sqrt
from typing import Callable, Iterable, Sequence


Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]


def _finite(values: Iterable[float], label: str) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if not all(isfinite(value) for value in result):
        raise ValueError(f"{label} must contain only finite values")
    return result


@dataclass(frozen=True)
class Pose:
    """Position in metres and quaternion in ROS ``xyzw`` order."""

    position: Vector3
    orientation_xyzw: Quaternion

    def __post_init__(self) -> None:
        position = _finite(self.position, "position")
        orientation = _finite(self.orientation_xyzw, "orientation")
        if len(position) != 3:
            raise ValueError("position must contain three values")
        if len(orientation) != 4:
            raise ValueError("orientation must contain four values")
        magnitude = sqrt(sum(value * value for value in orientation))
        if magnitude == 0.0:
            raise ValueError("orientation quaternion cannot be zero")
        object.__setattr__(self, "position", position)
        object.__setattr__(
            self,
            "orientation_xyzw",
            tuple(value / magnitude for value in orientation),
        )

    def translated_world_z(self, distance_m: float) -> "Pose":
        distance_m = float(distance_m)
        if not isfinite(distance_m):
            raise ValueError("world-Z offset must be finite")
        x, y, z = self.position
        return Pose((x, y, z + distance_m), self.orientation_xyzw)


class MotionKind(str, Enum):
    MOVE_J = "move_j"
    MOVE_L = "move_l"
    WAIT = "wait"
    GRIPPER_CLOSE = "gripper_close"
    GRIPPER_OPEN = "gripper_open"


@dataclass(frozen=True)
class MotionStep:
    kind: MotionKind
    target: Pose | None = None
    duration_s: float | None = None

    def __post_init__(self) -> None:
        is_move = self.kind in (MotionKind.MOVE_J, MotionKind.MOVE_L)
        if is_move != (self.target is not None):
            raise ValueError("only MoveJ/MoveL steps require a target pose")
        if self.kind is MotionKind.WAIT:
            if (
                self.duration_s is None
                or not isfinite(self.duration_s)
                or self.duration_s < 0.0
            ):
                raise ValueError("wait duration must be non-negative")
        elif self.duration_s is not None:
            raise ValueError("duration is only valid for wait steps")


@dataclass(frozen=True)
class BrickMotionTargets:
    """One brick's pickup and placement center poses in a shared frame."""

    brick_id: str
    pickup: Pose
    placement: Pose

    def __post_init__(self) -> None:
        if not self.brick_id.strip():
            raise ValueError("brick_id cannot be empty")


def rail_position(
    target: Pose,
    rail_origin_m: Vector3,
    rail_direction: Vector3,
    offset_m: float = 0.0,
    limits_m: tuple[float, float] = (0.0, 4.0),
) -> float:
    """Project a target onto the rail and clamp it to the configured limits.

    Grasshopper used ``-robot_root.YAxis`` as ``rail_direction``.  Making that
    direction explicit prevents silently assuming that convention for LL.
    """

    origin = _finite(rail_origin_m, "rail origin")
    direction = _finite(rail_direction, "rail direction")
    if len(origin) != 3 or len(direction) != 3:
        raise ValueError("rail origin and direction must contain three values")
    norm = sqrt(sum(value * value for value in direction))
    if norm == 0.0:
        raise ValueError("rail direction cannot be zero")
    unit = tuple(value / norm for value in direction)
    offset_m = float(offset_m)
    if not isfinite(offset_m):
        raise ValueError("rail offset must be finite")
    projected = sum(
        (target.position[index] - origin[index]) * unit[index]
        for index in range(3)
    ) + offset_m
    minimum, maximum = (float(value) for value in limits_m)
    if not isfinite(minimum) or not isfinite(maximum):
        raise ValueError("rail limits must be finite")
    if minimum > maximum:
        raise ValueError("rail limits are reversed")
    return min(max(projected, minimum), maximum)


def build_pick_place_steps(
    targets: BrickMotionTargets,
    retract_m: float,
    wait_s: float = 1.0,
    home: Pose | None = None,
) -> tuple[MotionStep, ...]:
    """Build the recovered GH sequence without solving TCP or IK.

    Lead-in and lead-out are the same world-Z-offset pose, matching scripts 2
    and 3.  A home step is included only when an explicit home pose is given.
    """

    retract_m = float(retract_m)
    if not isfinite(retract_m) or retract_m < 0.0:
        raise ValueError("retract distance must be a finite non-negative value")
    pickup_lead = targets.pickup.translated_world_z(retract_m)
    placement_lead = targets.placement.translated_world_z(retract_m)
    steps: list[MotionStep] = [
        MotionStep(MotionKind.MOVE_J, pickup_lead),
        MotionStep(MotionKind.MOVE_L, targets.pickup),
        MotionStep(MotionKind.WAIT, duration_s=wait_s),
        MotionStep(MotionKind.GRIPPER_CLOSE),
        MotionStep(MotionKind.WAIT, duration_s=wait_s),
        MotionStep(MotionKind.MOVE_L, pickup_lead),
    ]
    if home is not None:
        steps.append(MotionStep(MotionKind.MOVE_J, home))
    steps.extend(
        [
            MotionStep(MotionKind.MOVE_J, placement_lead),
            MotionStep(MotionKind.MOVE_L, targets.placement),
            MotionStep(MotionKind.WAIT, duration_s=wait_s),
            MotionStep(MotionKind.GRIPPER_OPEN),
            MotionStep(MotionKind.WAIT, duration_s=wait_s),
            MotionStep(MotionKind.MOVE_L, placement_lead),
        ]
    )
    return tuple(steps)


ReachabilityCheck = Callable[[Pose], tuple[bool, str | None]]


@dataclass(frozen=True)
class ReachabilityResult:
    brick_id: str
    reachable: bool
    reason: str | None = None


def classify_reachability(
    bricks: Sequence[BrickMotionTargets],
    check: ReachabilityCheck,
    retract_m: float,
) -> tuple[ReachabilityResult, ...]:
    """Keep reachability separate and inject the future TCP-aware solver."""

    results: list[ReachabilityResult] = []
    for brick in bricks:
        poses = (
            brick.pickup,
            brick.pickup.translated_world_z(retract_m),
            brick.placement,
            brick.placement.translated_world_z(retract_m),
        )
        for pose in poses:
            reachable, reason = check(pose)
            if not reachable:
                results.append(ReachabilityResult(brick.brick_id, False, reason))
                break
        else:
            results.append(ReachabilityResult(brick.brick_id, True))
    return tuple(results)
