"""Brick-frame to LL TCP/flange target conversion.

The local X-axis half-turn is the explicit operation recovered from GH scripts
2 and 3.  It is kept separate from the CAD flange-to-tool transform.
"""

from __future__ import annotations

from math import acos, cos, pi, sin, sqrt

from .motion import Pose, Vector3

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
    """PGF jaws span brick width (+Y); retain GH's downward tool Z."""

    return Pose(
        brick_pose.position,
        quaternion_multiply(brick_pose.orientation_xyzw,
                            quaternion_multiply(rpy_to_quaternion(0, 0, pi / 2), RX_PI)),
    )


def flange_target_from_tcp(tcp_pose: Pose, flange_tcp: Pose) -> Pose:
    """Compute flange pose from world TCP pose and flange-relative TCP pose."""

    flange_orientation = quaternion_multiply(
        tcp_pose.orientation_xyzw, quaternion_inverse(flange_tcp.orientation_xyzw))
    tool_translation_world = quaternion_rotate(flange_orientation, flange_tcp.position)
    flange_position = tuple(
        tcp_pose.position[index] - tool_translation_world[index]
        for index in range(3)
    )
    return Pose(flange_position, flange_orientation)


def brick_frame_from_tcp(tcp_pose: Pose) -> Pose:
    """Inverse of the single brick-to-PGF frame mapping (used for GH home)."""
    grasp = tcp_target_from_brick(Pose((0, 0, 0), (0, 0, 0, 1)))
    return Pose(tcp_pose.position, quaternion_multiply(
        tcp_pose.orientation_xyzw, quaternion_inverse(grasp.orientation_xyzw)))


def quaternion_slerp(a: Quaternion, b: Quaternion, t: float) -> Quaternion:
    """Shortest quaternion interpolation, as in the GH MoveL visualizer."""
    dot = sum(x*y for x,y in zip(a,b))
    if dot < 0:
        b = tuple(-v for v in b)
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        q = tuple(x + t*(y-x) for x,y in zip(a,b))
        norm = sqrt(sum(v*v for v in q))
        return tuple(v/norm for v in q)
    theta = acos(dot)
    return tuple((sin((1-t)*theta)*x + sin(t*theta)*y)/sin(theta) for x,y in zip(a,b))
