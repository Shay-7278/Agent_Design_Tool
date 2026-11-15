"""Brick-frame to LL TCP/flange target conversion.

The local X-axis half-turn is the explicit operation recovered from GH scripts
2 and 3.  It is kept separate from the CAD flange-to-tool transform.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, isfinite, sin, sqrt
from typing import Callable, Sequence

from .motion import MotionKind, MotionStep, Pose, Vector3

Quaternion = tuple[float, float, float, float]
JointVector = tuple[float, float, float, float, float, float]


def quaternion_multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quaternion_inverse(q: Quaternion) -> Quaternion:
    x, y, z, w = q
    norm = x * x + y * y + z * z + w * w
    if norm == 0.0:
        raise ValueError("cannot invert a zero quaternion")
    return (-x / norm, -y / norm, -z / norm, w / norm)


def quaternion_rotate(q: Quaternion, vector: Vector3) -> Vector3:
    pure = (vector[0], vector[1], vector[2], 0.0)
    rotated = quaternion_multiply(quaternion_multiply(q, pure), quaternion_inverse(q))
    return rotated[:3]


def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr, sr = cos(roll / 2.0), sin(roll / 2.0)
    cp, sp = cos(pitch / 2.0), sin(pitch / 2.0)
    cy, sy = cos(yaw / 2.0), sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


RX_PI: Quaternion = (1.0, 0.0, 0.0, 0.0)


def tcp_target_from_brick(brick_pose: Pose) -> Pose:
    """Apply the original GH local Rx(pi) to a brick frame."""

    return Pose(
        brick_pose.position,
        quaternion_multiply(brick_pose.orientation_xyzw, RX_PI),
    )


def flange_target_from_tcp(tcp_pose: Pose, flange_tcp: Pose) -> Pose:
    """Compute flange pose from world TCP pose and flange-relative TCP pose."""

    tool_translation_world = quaternion_rotate(
        tcp_pose.orientation_xyzw, flange_tcp.position
    )
    flange_position = tuple(
        tcp_pose.position[index] - tool_translation_world[index]
        for index in range(3)
    )
    flange_orientation = quaternion_multiply(
        tcp_pose.orientation_xyzw,
        quaternion_inverse(flange_tcp.orientation_xyzw),
    )
    return Pose(flange_position, flange_orientation)


@dataclass(frozen=True)
class LLTrajectoryPoint:
    track_m: float
    joints_rad: JointVector
    source_kind: MotionKind


IKSolver = Callable[[Pose], Sequence[float]]


def convert_motion_steps_to_ll(
    steps: Sequence[MotionStep],
    flange_tcp: Pose,
    ik_solver: IKSolver,
    rail_origin_m: Vector3,
    rail_direction: Vector3,
    rail_limits_m: tuple[float, float] = (0.0, 4.0),
) -> tuple[LLTrajectoryPoint, ...]:
    """Convert move steps to LL track + six-joint targets.

    The IK implementation is injected deliberately: this function will not
    claim a target is valid until the IRB4600 solver is supplied and checked.
    Non-move steps remain executor events and are not emitted as trajectory
    points.
    """

    from .motion import rail_position

    output: list[LLTrajectoryPoint] = []
    for step in steps:
        if step.kind not in (MotionKind.MOVE_J, MotionKind.MOVE_L):
            continue
        assert step.target is not None
        tcp_target = tcp_target_from_brick(step.target)
        flange_target = flange_target_from_tcp(tcp_target, flange_tcp)
        joints = tuple(float(value) for value in ik_solver(flange_target))
        if len(joints) != 6 or not all(isfinite(value) for value in joints):
            raise ValueError("IK solver must return six finite joint values")
        output.append(
            LLTrajectoryPoint(
                rail_position(tcp_target, rail_origin_m, rail_direction, limits_m=rail_limits_m),
                joints, step.kind,
            )
        )
    return tuple(output)
