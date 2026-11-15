#!/usr/bin/env python
from math import atan2, sqrt, radians, pi, cos, sin
import Rhino.Geometry as rg

__author__ = "Daniel Ruan, Arash Adel"
__copyright__ = "Copyright 2023, ADR Laboratory"
__credits__ = ["Daniel Ruan", "Arash Adel"]
__license__ = "MIT"
__email__ = "daniel.ruan@princeton.edu"

class RapidDataType:
    def __init__(self, name=None, type=None):
        self.name = name
        self.type = type
        self.declared = False

    def value(self):
        raise NotImplementedError

    def __str__(self):
        if self.declared:
            return self.name
        return self.value()

class Pos(RapidDataType):
    def __init__(self, x=0, y=0, z=0, name=None):
        RapidDataType.__init__(self, name, "pos")
        self.x = x
        self.y = y
        self.z = z

    @classmethod
    def from_rh_point3d(cls, pt):
        return cls(pt.X, pt.Y, pt.Z)

    def to_rh_point3d(self):
        return rg.Point3d(self.x, self.y, self.z)

    def value(self):
        return "[%s, %s, %s]" % (self.x, self.y, self.z)


class Orient(RapidDataType):
    def __init__(self, q1=1, q2=0, q3=0, q4=0, name=None):
        RapidDataType.__init__(self, name, "orient")
        self.q1 = q1
        self.q2 = q2
        self.q3 = q3
        self.q4 = q4

    @classmethod
    def from_rh_quaternion(cls, qt):
        return cls(qt.A, qt.B, qt.C, qt.D)

    def to_rh_quaternion(self):
        return rg.Quaternion(self.q1, self.q2, self.q3, self.q4)

    def to_rh_rotation_matrix(self):
        A = rg.Transform.Identity
        A.M00 = 1 - 2 * (self.q3**2 + self.q4**2)
        A.M01 = 2 * (self.q2 * self.q3 - self.q1 * self.q4)
        A.M02 = 2 * (self.q2 * self.q4 + self.q1 * self.q3)

        A.M10 = 2 * (self.q2 * self.q3 + self.q1 * self.q4)
        A.M11 = 1 - 2 * (self.q2**2 + self.q4**2)
        A.M12 = 2 * (self.q3 * self.q4 - self.q1 * self.q2)

        A.M20 = 2 * (self.q2 * self.q4 - self.q1 * self.q3)
        A.M21 = 2 * (self.q3 * self.q4 + self.q1 * self.q2)
        A.M22 = 1 - 2 * (self.q2**2 + self.q3**2)

        A.M33 = 1.0

        return A

    def value(self):
        return "[%s, %s, %s, %s]" % (self.q1, self.q2, self.q3, self.q4)


class ConfData(RapidDataType):
    def __init__(self, cf1=0, cf4=0, cf6=0, cfx=0, name=None):
        RapidDataType.__init__(self, name, "confdata")
        self.cf1 = cf1
        self.cf4 = cf4
        self.cf6 = cf6
        self.cfx = cfx

    def value(self):
        return "[%s, %s, %s, %s]" % (self.cf1, self.cf4, self.cf6, self.cfx)


class Pose(RapidDataType):
    def __init__(self, trans=Pos(), rot=Orient(), name=None):
        RapidDataType.__init__(self, name, "pose")
        self.trans = trans
        self.rot = rot

    @classmethod
    def from_rh_plane(cls, pln):
        trans = Pos.from_rh_point3d(pln.Origin)

        A = rg.Transform.ChangeBasis(pln, rg.Plane.WorldXY)

        trace = A.M00 + A.M11 + A.M22

        if trace > 0:
            s = sqrt(1 + trace) * 2
            q1 = 0.25 * s
            q2 = (A.M21 - A.M12) / s
            q3 = (A.M02 - A.M20) / s
            q4 = (A.M10 - A.M01) / s
        elif (A.M00 > A.M11) and (A.M00 > A.M22):
            s = sqrt(1 + A.M00 - A.M11 - A.M22) * 2
            q1 = (A.M21 - A.M12) / s
            q2 = 0.25 * s
            q3 = (A.M01 + A.M10) / s
            q4 = (A.M02 + A.M20) / s
        elif A.M11 > A.M22:
            s = sqrt(1 + A.M11 - A.M00 - A.M22) * 2
            q1 = (A.M02 - A.M20) / s
            q2 = (A.M01 + A.M10) / s
            q3 = 0.25 * s
            q4 = (A.M12 + A.M21) / s
        else:
            s = sqrt(1 + A.M22 - A.M00 - A.M11) * 2
            q1 = (A.M10 - A.M01) / s
            q2 = (A.M02 + A.M20) / s
            q3 = (A.M12 + A.M21) / s
            q4 = 0.25 * s
        rot = Orient.from_rh_quaternion(rg.Quaternion(q1, q2, q3, q4))

        return cls(trans, rot)

    def to_rh_plane(self):
        rg_trans = rg.Transform.Translation(rg.Vector3d(
                   self.trans.to_rh_point3d()))
        rg_rot = self.rot.to_rh_rotation_matrix()
        xform = rg_trans * rg_rot
        pln = rg.Plane.WorldXY
        pln.Transform(xform)

        return pln

    def value(self):
        return "[ %s, %s ]" % (self.trans, self.rot)


class RobJoint(RapidDataType):
    def __init__(self, rax_1="0", rax_2="0", rax_3="0", rax_4="0",
                 rax_5="0", rax_6="0", name=None):
        RapidDataType.__init__(self, name, "robjoint")
        self.rax_1 = rax_1
        self.rax_2 = rax_2
        self.rax_3 = rax_3
        self.rax_4 = rax_4
        self.rax_5 = rax_5
        self.rax_6 = rax_6

    @classmethod
    def from_list(cls, joints):
        if len(joints) != 6:
            raise ValueError("Provided joint list is not length 6!")
        return cls(joints[0], joints[1], joints[2], joints[3], joints[4],
                   joints[5])

    def to_list(self):
        return [self.rax_1, self.rax_2, self.rax_3, self.rax_4, self.rax_5,
                self.rax_6]

    def value(self):
        return "[ %s, %s, %s, %s, %s, %s]" % (self.rax_1, self.rax_2,
                                              self.rax_3, self.rax_4,
                                              self.rax_5, self.rax_6)


class ExtJoint(RapidDataType):
    def __init__(self, eax_a="9E9", eax_b="9E9", eax_c="9E9", eax_d="9E9",
                 eax_e="9E9", eax_f="9E9", name=None):
        RapidDataType.__init__(self, name, "extjoint")
        self.eax_a = eax_a
        self.eax_b = eax_b
        self.eax_c = eax_c
        self.eax_d = eax_d
        self.eax_e = eax_e
        self.eax_f = eax_f

    def value(self):
        return "[ %s, %s, %s, %s, %s, %s]" % (self.eax_a, self.eax_b,
                                              self.eax_c, self.eax_d,
                                              self.eax_e, self.eax_f)


class RobTarget(RapidDataType):
    def __init__(self, trans=Pos(), rot=Orient(), robconf=ConfData(),
                 extax=ExtJoint(), name=None):
        RapidDataType.__init__(self, name, "robtarget")
        self.trans = trans
        self.rot = rot
        self.robconf = robconf
        self.extax = extax

    def value(self):
        return "[ %s, %s, %s, %s ]" % (self.trans, self.rot, self.robconf,
                                       self.extax)

    @classmethod
    def from_rh_plane(cls, pln, robconf=ConfData(), extax=ExtJoint()):
        pose = Pose.from_rh_plane(pln)
        return cls(pose.trans, pose.rot, robconf, extax)

    def to_rh_plane(self):
        pose = self.get_pose()
        return pose.to_rh_plane()

    def get_rh_point3d(self):
        return self.trans.to_rh_point3d()

    def get_rh_quaternion(self):
        return self.rot.to_rh_quaternion()

    def get_pose(self):
        return Pose(self.trans, self.rot)

    def to_dict(self):
        # Dictionary format utilized for IK calculation
        # dict{'trans': xyz translation
        #      'rot': quaternion rotation
        #      'robconf': robot configuration}
        dict = {
            'trans': {
                'x': self.trans.x,
                'y': self.trans.y,
                'z': self.trans.z},
            'rot': {
                'q1': self.rot.q1,
                'q2': self.rot.q2,
                'q3': self.rot.q3,
                'q4': self.rot.q4},
            'robconf': {
                'cf1': self.robconf.cf1,
                'cf4': self.robconf.cf4,
                'cf6': self.robconf.cf6,
                'cfx': self.robconf.cfx}
        }

        return dict

class JointTarget(RapidDataType):
    def __init__(self, robax=RobJoint(), extax=ExtJoint(), name=None):
        RapidDataType.__init__(self, name, "jointtarget")
        self.robax = robax
        self.extax = extax

    def value(self):
        return "[ %s, %s ]" % (self.robax, self.extax)


class SpeedData(RapidDataType):
    def __init__(self, v_tcp, v_ori=500, v_leax=5000, v_reax=1000, name=None):
        RapidDataType.__init__(self, name, "speeddata")
        self.v_tcp = v_tcp
        self.v_ori = v_ori
        self.v_leax = v_leax
        self.v_reax = v_reax
        self.predefined_name = None

    @classmethod
    def predefined(cls, name):
        # TODO: Does not currently match exact name values (e.g., v12 would
        # match, but is not valid)
        if re.match(r"^v\d+$", name) is None:
            raise ValueError("Invalid predefined speeddata name", name)
        v = cls(int(name[1:]))
        v.predefined_name = name
        return v

    def value(self):
        if self.predefined_name is not None:
            return self.predefined_name
        return "[ %s, %s, %s, %s ]" % (self.v_tcp, self.v_ori, self.v_leax,
                                       self.v_reax)


class ZoneData(RapidDataType):
    def __init__(self, finep, pzone_tcp, pzone_ori, pzone_eax, zone_ori,
                 zone_leax, zone_reax, name=None):
        RapidDataType.__init__(self, name, "zonedata")
        self.finep = finep
        self.pzone_tcp = pzone_tcp
        self.pzone_ori = pzone_ori
        self.pzone_eax = pzone_eax
        self.zone_ori = zone_ori
        self.zone_leax = zone_leax
        self.zone_reax = zone_reax
        self.predefined_name = None

    @classmethod
    def predefined(cls, name):
        # TODO: Does not currently match exact name values (e.g., z12 would
        # match, but is not valid)
        if re.match(r"^(z\d+|fine)$", name) is None:
            raise ValueError("Invalid predefined zonedata name", name)

        if name.lower() == "fine":
            z = cls(True, 0, 0, 0, 0, 0, 0)
            z.predefined_name = "fine"
            return z

        n = int(name[1:])
        if n == 0:
            z = cls(False, 0.3, 0.3, 0.3, 0.03, 0.3, 0.03)
            z.predefined_name = "z0"
        elif n == 1:
            z = cls(False, 1, 1, 1, 0.1, 1, 0.1)
            z.predefined_name = "z1"
        else:
            m = ceil(n * 1.5)
            z = cls(False, n, m, m, m / 10.0, m, m / 10.0)
            z.predefined_name = name
        return z

    def value(self):
        if self.predefined_name is not None:
            return self.predefined_name
        return "[ %s, %s, %s, %s, %s, %s, %s ]" \
            % (str(self.finep).upper(), self.pzone_tcp, self.pzone_ori,
               self.pzone_eax, self.zone_ori, self.zone_leax, self.zone_reax)


class LoadData(RapidDataType):
    def __init__(self, mass=0, cog=Pos(), aom=Orient(), ix=0, iy=0, iz=0,
                 name=None):
        RapidDataType.__init__(self, name, "loaddata")
        self.mass = mass
        self.cog = cog
        self.aom = aom
        self.ix = ix
        self.iy = iy
        self.iz = iz

    def value(self):
        return "[%s, %s, %s, %s, %s, %s]" % (self.mass, self.cog, self.aom,
                                             self.ix, self.iy, self.iz)

class ToolData(RapidDataType):
    def __init__(self, robhold=True, tframe=Pose(), tload=LoadData(),
                 name=None):
        RapidDataType.__init__(self, name, "tooldata")
        self.robhold = robhold
        self.tframe = tframe
        self.tload = tload

    def value(self):
        return "[%s, %s, %s]" % (str(self.robhold).upper(), self.tframe,
                                 self.tload)


DH_IRB4600 = {'theta': [0,    0,  -90,   0,    0,   0,   0],
              'alpha': [0,  -90,    0, -90,   90, -90,   0],
              'a':     [0,  175, 1095, 175,    0,   0,   0],
              'd':   [495,    0,    0,   0, 1270,   0, 135]}


def joint_to_joint_transform(joint, theta):
    """Compute the successive transform from joint i-1 to i.

    Parameters
    ----------
    joint: int
        Robot joint number, starting from index 1. Robot root is joint 0.
    theta: float
        Angle of rotation, in radians

    Return
    ----------
    A: Rhino.Geometry.Transform
        4x4 transform matrix

    """
    DH = DH_IRB4600
    theta_offset = radians(DH['theta'][joint])
    theta = theta + theta_offset
    alpha = radians(DH['alpha'][joint])
    a = DH['a'][joint]
    d = DH['d'][joint]
    A = rg.Transform.ZeroTransformation
    A.M00 = cos(theta)
    A.M01 = -cos(alpha)*sin(theta)
    A.M02 = sin(alpha)*sin(theta)
    A.M03 = a*cos(theta)
    A.M10 = sin(theta)
    A.M11 = cos(alpha)*cos(theta)
    A.M12 = -sin(alpha)*cos(theta)
    A.M13 = a*sin(theta)
    A.M21 = sin(alpha)
    A.M22 = cos(alpha)
    A.M23 = d

    return A


def fk_n(thetas):
    """Compute the partial forward kinematics transform for a given list of
    robot joint angles, used in inverse kinematics.

    Returns transform from world to the nth link's D-H coordinate system.

    Parameters
    ----------
    thetas: list[float]
        List of joint angles [theta_1, theta_2, ..., theta_n].
    root: Rhino.Geometry.Plane
        Robot root coordinate plane relative to world.

    Return
    ----------
    transform: Rhino.Geometry.Transform
        4x4 transform matrix.

    """
    transform = rg.Transform.Identity
    for i, theta in enumerate(thetas):
        A = joint_to_joint_transform(i + 1, theta)
        transform = transform * A
    return transform


def fk(thetas, root=rg.Plane.WorldXY):
    """Compute the full forward kinematics for a given list of robot joint
    angles and robot root.

    Parameters
    ----------
    thetas: list[float]
        List of joint angles [theta_1, theta_2, ..., theta_6].
    root: Rhino.Geometry.Plane
        Robot root coordinate plane relative to world.

    Return
    ----------
    transforms: list[Rhino.Geometry.Transform]
        List of 4x4 transformation matrices corresponding to each link,
        including the robot base, relative to the world coordinate system.

    """
    # Robot base link transform
    partial_transform = (rg.Transform.ChangeBasis(root, rg.Plane.WorldXY)
                         * joint_to_joint_transform(0, 0))
    transforms = [partial_transform]
    for i, theta in enumerate(thetas):
        A = joint_to_joint_transform(i + 1, theta)
        partial_transform = partial_transform * A
        transforms.append(partial_transform)

    return transforms


def get_flange_root(target_plane, root=rg.Plane.WorldXY,
                    tool_tcp=rg.Plane.WorldXY):
    """Return the flange target in the robot's (possibly moved) DH root frame.

    Target TCP is in world coordinates.  Tool TCP is defined relative to the
    flange.  The correct relation is flange = target * inverse(tool TCP).
    This matches the corrected IRB4600 animation transform chain.
    """
    target_transform = rg.Transform.ChangeBasis(target_plane, rg.Plane.WorldXY)
    tool_transform = rg.Transform.ChangeBasis(tool_tcp, rg.Plane.WorldXY)
    success, inverse_tool = tool_transform.TryGetInverse()
    if not success:
        raise ValueError("Tool TCP transform cannot be inverted")

    flange_plane_world = rg.Plane.WorldXY
    flange_plane_world.Transform(target_transform * inverse_tool)

    dh_transform = joint_to_joint_transform(0, 0)
    success, dh_inverse = dh_transform.TryGetInverse()
    if not success:
        raise ValueError("Robot base DH transform cannot be inverted")

    world_to_root = rg.Transform.ChangeBasis(rg.Plane.WorldXY, root)
    flange_plane_root = rg.Plane(flange_plane_world)
    flange_plane_root.Transform(dh_inverse * world_to_root)
    return flange_plane_root


def get_tcp_plane_world(flange_transform, root=rg.Plane.WorldXY,
                        tool_tcp=rg.Plane.WorldXY):
    root_transform = rg.Transform.ChangeBasis(root, rg.Plane.WorldXY)

    tcp_plane = tool_tcp.Clone()
    tcp_plane.Transform(root_transform * flange_transform)

    return tcp_plane


def ik(target, DH=DH_IRB4600):
    """Compute the inverse kinematics for a target robot pose.

    Utilizes a closed form geometric approach published in "Inverse Kinematic
    Analysis of IRB120 Robot Arm" (Oguz Faik Seven and Arif Ankarali, 2019).

    Parameters
    ----------
        target : rapid_types.RobTarget

    Return
    ----------
        solution : list[double]
            Robot joint angles in radians.

    """
    solution = []

    # See cfx values in RAPID technical reference manual, confdata
    # elbow:
    #   ABOVE_ARM = -1
    #   BELOW_ARM = 1
    # wrist:
    #     WRIST_DOWN = 1
    #     WRIST_UP = -1
    elbow = -1
    if target.robconf.cfx in {2, 3, 4, 5}:
        elbow = 1
    wrist = 1
    if target.robconf.cfx in {1, 3, 5, 7}:
        wrist = -1

    # Calculate approach vector from quaternion orientation
    end_effector_plane = target.to_rh_plane()
    normal_vec = end_effector_plane.XAxis
    sliding_vec = end_effector_plane.YAxis
    approach_vec = end_effector_plane.ZAxis

    # Calculate coordinates of fourth coordinate system
    p4 = end_effector_plane.Origin - DH['d'][6] * approach_vec

    # Joint 1 solution
    theta_1 = atan2(p4.Y, p4.X) - radians(DH['theta'][1])
    solution.append(theta_1)

    # IRB4600 case where joint 2 is offset along joint 1 x axis
    p4.X -= DH['a'][1] * cos(theta_1)
    p4.Y -= DH['a'][1] * sin(theta_1)

    # Joint 2 solution
    r = sqrt((p4.X ** 2) + (p4.Y ** 2))
    R = sqrt((r ** 2) + ((p4.Z - DH['d'][1]) ** 2))
    sin_alpha = (p4.Z - DH['d'][1]) / R
    cos_alpha = r / R
    cos_beta = (((DH['a'][2] ** 2) + (R ** 2) - (DH['a'][3] ** 2)
                 - (DH['d'][4] ** 2)) / (2 * DH['a'][2] * R))
    sin_beta = sqrt(1 - (cos_beta ** 2))
    alpha = atan2(sin_alpha, cos_alpha)
    beta = atan2(sin_beta, cos_beta)
    solution.append(-alpha + elbow * beta - radians(DH['theta'][2]))

    # Joint 3 solution
    cos_psi = (((DH['a'][2] ** 2) + (DH['a'][3] ** 2) + (DH['d'][4] ** 2)
                - (R ** 2)) / (2 * DH['a'][2] * sqrt((DH['a'][3] ** 2)
                                                     + (DH['d'][4] ** 2))))
    sin_psi = sqrt(1 - (cos_psi ** 2))
    cos_phi = DH['a'][3] / sqrt((DH['a'][3] ** 2) + (DH['d'][4] ** 2))
    sin_phi = DH['d'][4] / sqrt((DH['a'][3] ** 2) + (DH['d'][4] ** 2))
    psi = atan2(sin_psi, cos_psi)
    phi = atan2(sin_phi, cos_phi)
    solution.append(-phi - elbow * (pi - psi) - radians(DH['theta'][3]))

    # Joint 4 solution
    c3 = fk_n(solution)
    c3_x = rg.Vector3d(c3.M00, c3.M10, c3.M20)
    c3_y = rg.Vector3d(c3.M01, c3.M11, c3.M21)
    c3_z = rg.Vector3d(c3.M02, c3.M12, c3.M22)

    z4 = -rg.Vector3d.CrossProduct(c3_z, approach_vec)  # Flipped z4 direction
    z4.Unitize()
    if abs(sliding_vec * z4) < 0.00001:  # For rounding errors
        omega = normal_vec * z4
    else:
        omega = sliding_vec * z4

    if omega <= 0:  # Flipped z4 direction
        M = wrist * 1
    else:
        M = wrist * -1

    sin_rax_4 = -M * (z4 * c3_x)
    cos_rax_4 = M * (z4 * c3_y)
    # Prevent 180 deg flip - need to fix better
    #   Current implementation is discontinuous and can suddenly flip
    #   mid-trajectory
    if cos_rax_4 < 0:
        cos_rax_4 = abs(cos_rax_4)
        sin_rax_4 = -sin_rax_4
    sol_4 = atan2(sin_rax_4, cos_rax_4) - radians(DH['theta'][4])
    solution.append(sol_4)

    # Joint 5 solution
    c4 = fk_n(solution)
    c4_x = -rg.Vector3d(c4.M00, c4.M10, c4.M20) # Flipped z4 direction
    c4_y = -rg.Vector3d(c4.M01, c4.M11, c4.M21) # Flipped z4 direction

    sin_rax_5 = approach_vec * c4_x
    cos_rax_5 = -approach_vec * c4_y
    solution.append(atan2(sin_rax_5, cos_rax_5) - radians(DH['theta'][5]))

    # Joint 6 solution
    c5 = fk_n(solution)
    c5_y = rg.Vector3d(c5.M01, c5.M11, c5.M21)

    sin_rax_6 = normal_vec * c5_y
    cos_rax_6 = sliding_vec * c5_y
    solution.append(atan2(sin_rax_6, cos_rax_6) - radians(DH['theta'][6]))

    return solution




def get_placement_frames(brick, z_retract):
    """Brick center -> TCP by local Rx(pi), plus a world-Z approach frame."""
    brick_frame = rg.Plane(brick.Plane)
    brick_frame.Origin = brick.Center

    placement_frame = rg.Plane(brick_frame)
    placement_frame.Rotate(pi, brick_frame.XAxis)

    offset_placement_frame = rg.Plane(placement_frame)
    offset_placement_frame.Translate(rg.Vector3d.ZAxis * z_retract)
    return [placement_frame, offset_placement_frame]


def get_track_position(target_plane, root, track_offset,
                       minimum=0.0, maximum=4000.0):
    """Same rail convention as RobTarget.set_track_offset in the IRB4600 code.

    Rail moves along -robot_root.YAxis. The scalar track_offset is an additive
    offset to the projected target location, NOT an absolute world-Y value.
    """
    rail_direction = -root.YAxis
    delta = rg.Vector3d(target_plane.Origin - root.Origin)
    position = delta * rail_direction + track_offset
    return min(max(position, minimum), maximum)


def main():
    global bricks, z_retract, track_offset
    global robot_root, tool
    global reachable, unreachable

    if robot_root is None:
        robot_root = rg.Plane.WorldXY
    if z_retract is None:
        z_retract = 0.0
    if track_offset is None:
        track_offset = 0.0
    if tool is None:
        tool = ToolData()
    if bricks is None:
        bricks = []

    z_retract = float(z_retract)
    track_offset = float(track_offset)
    tcp_relative_to_flange = tool.tframe.to_rh_plane()

    reachable = []
    unreachable = []
    reasons = []

    for brick_index, brick in enumerate(bricks):
        try:
            # One rail position per brick (held fixed for place + retract).
            frames = get_placement_frames(brick, z_retract)
            track_position = get_track_position(frames[0], robot_root,
                                                track_offset)
            moving_root = rg.Plane(robot_root)
            moving_root.Translate(-track_position * moving_root.YAxis)

            for frame in frames:
                flange_frame = get_flange_root(frame, moving_root,
                                               tcp_relative_to_flange)
                rob_target = RobTarget.from_rh_plane(flange_frame)
                joints = ik(rob_target)
                if len(joints) != 6 or not all(math_isfinite(x) for x in joints):
                    raise ValueError("Invalid/nonfinite IK joint solution")

        except (ValueError, ZeroDivisionError, OverflowError, ArithmeticError) as exc:
            unreachable.append(brick)
            reasons.append("Brick {}: {}".format(brick_index, exc))
        else:
            reachable.append(brick)

    print("Reachability: {} reachable / {} unreachable".format(
          len(reachable), len(unreachable)))
    print("Track: -robot_root.YAxis; additive offset {}; limits 0..4000".format(
          track_offset))
    for reason in reasons[:5]:
        print(reason)


def math_isfinite(value):
    # Avoid requiring numpy; compatible with older Grasshopper Python.
    return not (math_isnan(value) or abs(value) == float('inf'))


def math_isnan(value):
    return value != value


if __name__ == "__main__":
    main()
