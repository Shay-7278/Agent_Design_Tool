"""Rhino-free IRB4600 DH forward kinematics and reachability primitives.

The dimensions and joint offsets are the values extracted from the GH graph.
This module deliberately exposes FK and a solver boundary; it does not claim
an IK solution until the target is checked against the robot's URDF limits.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, isfinite, pi, sin, sqrt
from typing import Sequence

from .motion import Pose


DH_THETA = (0.0, 0.0, -pi / 2.0, 0.0, 0.0, 0.0, 0.0)
DH_ALPHA = (0.0, -pi / 2.0, 0.0, -pi / 2.0, pi / 2.0, -pi / 2.0, 0.0)
DH_A = (0.0, 0.175, 1.095, 0.175, 0.0, 0.0, 0.0)
DH_D = (0.495, 0.0, 0.0, 0.0, 1.270, 0.0, 0.135)


Matrix4 = tuple[tuple[float, float, float, float], ...]


def _matmul(left: Matrix4, right: Matrix4) -> Matrix4:
    return tuple(
        tuple(sum(left[row][k] * right[k][column] for k in range(4)) for column in range(4))
        for row in range(4)
    )


def _dh(joint: int, angle: float) -> Matrix4:
    theta = angle + DH_THETA[joint]
    alpha = DH_ALPHA[joint]
    ct, st, ca, sa = cos(theta), sin(theta), cos(alpha), sin(alpha)
    return (
        (ct, -ca * st, sa * st, DH_A[joint] * ct),
        (st, ca * ct, -sa * ct, DH_A[joint] * st),
        (0.0, sa, ca, DH_D[joint]),
        (0.0, 0.0, 0.0, 1.0),
    )


def forward_kinematics(joints_rad: Sequence[float]) -> Matrix4:
    if len(joints_rad) != 6:
        raise ValueError("IRB4600 FK requires six arm joints")
    transform = _dh(0, 0.0)
    for joint, angle in enumerate(joints_rad, start=1):
        transform = _matmul(transform, _dh(joint, float(angle)))
    return transform


def _rotation_columns(pose: Pose) -> tuple[tuple[float, float, float], ...]:
    x, y, z, w = pose.orientation_xyzw
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )


def _column(rotation: tuple[tuple[float, float, float], ...], index: int) -> tuple[float, float, float]:
    return tuple(rotation[row][index] for row in range(3))


def _dot(a: Sequence[float], b: Sequence[float]) -> float:
    return sum(x * y for x, y in zip(a, b))


def _cross(a: Sequence[float], b: Sequence[float]) -> tuple[float, float, float]:
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _unit(vector: Sequence[float]) -> tuple[float, float, float]:
    length = sqrt(_dot(vector, vector))
    if length <= 1e-12:
        raise ValueError("zero-length IK vector")
    return tuple(value / length for value in vector)


def _clamped_sqrt(value: float) -> float:
    if value < -1e-8 or value > 1.0 + 1e-8:
        raise ValueError("target is outside the IRB4600 geometric workspace")
    return sqrt(max(0.0, min(1.0, value)))


def inverse_kinematics(target: Pose, cfx: int = 0) -> tuple[float, ...]:
    """Port of the extracted IRB4600 closed-form GH solver.

    ``target`` is the flange pose in the IRB4600 DH root frame.  The caller is
    responsible for transforming the workcell target into that frame and for
    checking the returned solution against the URDF joint limits.
    """

    rotation = _rotation_columns(target)
    normal, sliding, approach = (_column(rotation, i) for i in range(3))
    p4 = [target.position[i] - DH_D[6] * approach[i] for i in range(3)]
    elbow = 1 if cfx in {2, 3, 4, 5} else -1
    wrist = -1 if cfx in {1, 3, 5, 7} else 1

    theta_1 = atan2(p4[1], p4[0]) - DH_THETA[1]
    p4[0] -= DH_A[1] * cos(theta_1)
    p4[1] -= DH_A[1] * sin(theta_1)
    radius = sqrt(p4[0] ** 2 + p4[1] ** 2)
    distance = sqrt(radius ** 2 + (p4[2] - DH_D[1]) ** 2)
    if distance <= 1e-12:
        raise ValueError("singular IRB4600 target")
    sin_alpha = (p4[2] - DH_D[1]) / distance
    cos_alpha = radius / distance
    cos_beta = ((DH_A[2] ** 2 + distance ** 2 - DH_A[3] ** 2 - DH_D[4] ** 2)
                / (2 * DH_A[2] * distance))
    beta = atan2(_clamped_sqrt(1 - cos_beta ** 2), cos_beta)
    solution = [
        theta_1,
        -atan2(sin_alpha, cos_alpha) + elbow * beta - DH_THETA[2],
    ]
    cos_psi = ((DH_A[2] ** 2 + DH_A[3] ** 2 + DH_D[4] ** 2 - distance ** 2)
               / (2 * DH_A[2] * sqrt(DH_A[3] ** 2 + DH_D[4] ** 2)))
    psi = atan2(_clamped_sqrt(1 - cos_psi ** 2), cos_psi)
    phi = atan2(DH_D[4], DH_A[3])
    solution.append(-phi - elbow * (pi - psi) - DH_THETA[3])

    c3 = forward_kinematics_partial(solution)
    c3_x, c3_y, c3_z = _column3(c3, 0), _column3(c3, 1), _column3(c3, 2)
    z4 = _unit(tuple(-value for value in _cross(c3_z, approach)))
    omega = _dot(normal, z4) if abs(_dot(sliding, z4)) < 1e-5 else _dot(sliding, z4)
    multiplier = wrist if omega <= 0 else -wrist
    sin4 = -multiplier * _dot(z4, c3_x)
    cos4 = multiplier * _dot(z4, c3_y)
    if cos4 < 0:
        cos4 = abs(cos4)
        sin4 = -sin4
    solution.append(atan2(sin4, cos4) - DH_THETA[4])

    c4 = forward_kinematics_partial(solution)
    c4_x = tuple(-value for value in _column3(c4, 0))
    c4_y = tuple(-value for value in _column3(c4, 1))
    solution.append(atan2(_dot(approach, c4_x), -_dot(approach, c4_y)) - DH_THETA[5])
    c5 = forward_kinematics_partial(solution)
    c5_y = _column3(c5, 1)
    solution.append(atan2(_dot(normal, c5_y), _dot(sliding, c5_y)) - DH_THETA[6])
    if len(solution) != 6 or not all(isfinite(value) for value in solution):
        raise ValueError("IRB4600 IK returned a nonfinite solution")
    return tuple(solution)


def forward_kinematics_partial(joints_rad: Sequence[float]) -> Matrix4:
    if len(joints_rad) > 6:
        raise ValueError("partial FK accepts at most six joints")
    transform = _dh(0, 0.0)
    for joint, angle in enumerate(joints_rad, start=1):
        transform = _matmul(transform, _dh(joint, float(angle)))
    return transform


def _column3(matrix: Matrix4, index: int) -> tuple[float, float, float]:
    return (matrix[0][index], matrix[1][index], matrix[2][index])


@dataclass(frozen=True)
class Reachability:
    reachable: bool
    reason: str | None = None


def validate_joint_vector(joints_rad: Sequence[float]) -> Reachability:
    if len(joints_rad) != 6:
        return Reachability(False, "IK result must contain six joints")
    if not all(float(joint) == float(joint) for joint in joints_rad):
        return Reachability(False, "IK result contains NaN")
    return Reachability(True)
