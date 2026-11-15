#!/usr/bin/env python

import re
import math
from math import ceil, sqrt, radians, cos, sin, atan2, pi


import Rhino.Geometry as rg
import ghpythonlib.treehelpers as th


__author__ = "Daniel Ruan"
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

    @classmethod
    def from_list(cls, l, name=None):
        return cls(l[0], l[1], l[2], l[3], name)

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
    def __init__(self, eax_a=9E9, eax_b=9E9, eax_c=9E9, eax_d=9E9,
                 eax_e=9E9, eax_f=9E9, name=None):
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

    def set_track_offset(self, target_plane, track_offset, root=rg.Plane.WorldXY, minimum=0.0, maximum=4000.0):
        direction = -root.YAxis #TODO: Check that this direction works for both robots
        track_pos = rg.Vector3d(target_plane.Origin - root.Origin) * direction + track_offset

        track_pos = min(max(track_pos, minimum), maximum)

        self.extax = ExtJoint(eax_a=track_pos)

    def get_ik(self):
        target = self
        if self.extax.eax_a != 9E9:
            from copy import deepcopy
            target = deepcopy(self)
            target.trans.y += self.extax.eax_a    # TODO: Fix for other track directions
        return ik(target)

    def check_reachable(self):
        try:
            self.get_ik()
        except ValueError:
            return False
        return True


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


def joint_to_joint_transform(joint, theta, DH=DH_IRB4600):
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
    """Get the flange frame relative to the robot root.

    Parameters
    ----------
    target_plane: Rhino.Geometry.Plane
        Target frame to transform, relative to world.
    root: Rhino.Geometry.Plane
        Robot root coordinate frame relative to world.
    tool_tcp: Rhino.Geometry.Plane
        Tool tcp frame relative to world.

    Return
    ----------
    flange_plane_root: Rhino.Geometry.Plane
        Flange frame relative to the robot root frame.

    """
    # TCP is defined in flange coordinates; IK needs flange = target * inverse(TCP).
    target_transform = rg.Transform.ChangeBasis(target_plane, rg.Plane.WorldXY)
    t_flange_tcp = rg.Transform.ChangeBasis(tool_tcp, rg.Plane.WorldXY)
    valid, t_tcp_flange = t_flange_tcp.TryGetInverse()
    if not valid:
        raise ValueError("Tool TCP transform is not invertible")
    flange_plane_world = rg.Plane.WorldXY
    flange_plane_world.Transform(target_transform * t_tcp_flange)

    flange_plane_root = flange_plane_world.Clone()
    DH_transform = joint_to_joint_transform(0, 0)
    _, DH_transform_inv = DH_transform.TryGetInverse()
    root_transform = rg.Transform.ChangeBasis(rg.Plane.WorldXY, root)
    flange_plane_root.Transform(DH_transform_inv * root_transform)

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


class RapidInstruction:
    def __init__(self):
        pass

    def __str__(self):
        raise NotImplementedError


class RapidInstruction:
    def __init__(self):
        pass

    def __str__(self):
        raise NotImplementedError


class MoveInstruction(RapidInstruction):
    def __init__(self, speed, zone, tool):
        RapidInstruction.__init__(self)
        self.speed = speed
        self.zone = zone
        self.tool = tool

    def __str__(self):
        raise NotImplementedError

    def get_rh_plane(self, robot_root):
        raise NotImplementedError

    def get_rh_point3d(self, robot_root):
        raise NotImplementedError

    def get_rh_quaternion(self, robot_root):
        raise NotImplementedError

    def get_eax_a(self):
        raise NotImplementedError


class MoveL(MoveInstruction):
    def __init__(self, to_point, speed, zone, tool):
        MoveInstruction.__init__(self, speed, zone, tool)
        self.to_point = to_point  # rapid_types.RobTarget

    def __str__(self):
        return "MoveL %s, %s, %s, %s;" % (self.to_point, self.speed, self.zone,
                                          self.tool)

    def get_rh_plane(self, robot_root):
        return self.to_point.to_rh_plane()

    def get_rh_point3d(self, robot_root):
        return self.to_point.get_rh_point3d()

    def get_rh_quaternion(self, robot_root):
        return self.to_point.get_rh_quaternion()

    def get_eax_a(self):
        return self.to_point.extax.eax_a


class MoveJ(MoveInstruction):
    def __init__(self, to_point, speed, zone, tool):
        MoveInstruction.__init__(self, speed, zone, tool)
        self.to_point = to_point  # rapid_types.RobTarget

    def __str__(self):
        return "MoveJ %s, %s, %s, %s;" % (self.to_point, self.speed, self.zone,
                                          self.tool)

    def get_rh_plane(self, robot_root):
        return self.to_point.to_rh_plane()

    def get_rh_point3d(self, robot_root):
        return self.to_point.get_rh_point3d()

    def get_rh_quaternion(self, robot_root):
        return self.to_point.get_rh_quaternion()

    def get_eax_a(self):
        return self.to_point.extax.eax_a


class MoveAbsJ(MoveInstruction):
    def __init__(self, to_joint_pos, speed, zone, tool):
        MoveInstruction.__init__(self, speed, zone, tool)
        self.to_joint_pos = to_joint_pos    # rapid_types.JointTarget

    def __str__(self):
        return "MoveAbsJ %s, %s, %s, %s;" % (self.to_joint_pos, self.speed,
                                             self.zone, self.tool)

    def get_rh_plane(self, robot_root):
        flange_transform = fk(self.to_joint_pos.robax.to_list())[-1]
        return get_tcp_plane_world(flange_transform, robot_root,
                                      self.tool.tframe.to_rh_plane())

    def get_rh_point3d(self, robot_root):
        plane = self.get_rh_plane(robot_root)
        return plane.Origin

    def get_rh_quaternion(self, robot_root):
        plane = self.get_rh_plane(robot_root)
        return Pose.from_rh_plane(plane).rot.to_rh_quaternion()

    def get_eax(self, i, robot_root):
        raise NotImplementedError


def quaternion_slerp(a, b, t):
    a.Unitize()
    b.Unitize()

    if a == b:
        return a

    dot_product = a.A * b.A + a.B * b.B + a.C * b.C + a.D * b.D

    # If the dot product is negative, invert one of the quaternions to take
    # the shortest path along the unit sphere
    if dot_product < 0:
        b = b.Conjugate
        b.A = -b.A
        dot_product = -dot_product

    # Clamp the dot product to the range [-1, 1] to avoid numerical instability
    dot_product = max(-1.0, min(1.0, dot_product))

    # Calculate the angle between the two quaternions
    theta = math.acos(dot_product)

    # Calculate the slerp interpolation factors
    sin_theta = math.sin(theta)
    factor_a = math.sin((1 - t) * theta) / sin_theta
    factor_b = math.sin(t * theta) / sin_theta

    # Interpolate between the quaternions
    result = a * factor_a + b * factor_b

    return result


def lerp(a, b, t):
    return a + ((b - a) * t)


def plane_from_pt_q(pt, quaternion):
    plane = rg.Plane.WorldXY
    orient = Orient.from_rh_quaternion(quaternion)
    plane.Transform(orient.to_rh_rotation_matrix())
    plane.Translate(rg.Vector3d(pt))

    return plane

class Toolpath():
    def __init__(self, instructions, robot_root):
        self.instructions = instructions
        self.robot_root = robot_root
        self.toolpath = self.generate_toolpath()

    def generate_toolpath(self):
        self.pts = []
        self.quaternions = []
        self.tools = []
        self.colors = []
        self.move_types = []
        self.track_pos = []

        # Brick display bookkeeping, adapted from the working IRB120 script.
        # No RAPID instructions or robot motion logic are modified.
        self.pickup_locations = []
        self.place_locations = []
        self.brick_events = []
        self.brick_command_samples = []
        self.grip_commands = 0
        self.release_commands = 0
        for item in self.instructions:
            # Take the last movement target as the brick frame at a grip event.
            # Handle both normal Python strings and Grasshopper Goo values.
            raw = getattr(item, "Value", item)
            command = str(raw).strip().lower().replace(" ", "").replace("\t", "")
            if command in ("setll_gripper;", "setll_gripper"):
                self.grip_commands += 1
                if self.pts:
                    frame = plane_from_pt_q(self.pts[-1], self.quaternions[-1])
                    index = len(self.pts) - 1
                    self.pickup_locations.append((index, frame))
                    self.brick_events.append((index, True))
            elif command in ("resetll_gripper;", "resetll_gripper"):
                self.release_commands += 1
                if self.pts:
                    frame = plane_from_pt_q(self.pts[-1], self.quaternions[-1])
                    index = len(self.pts) - 1
                    self.place_locations.append((index, frame))
                    self.brick_events.append((index, False))
            elif len(self.brick_command_samples) < 8 and not hasattr(item, "get_rh_point3d"):
                self.brick_command_samples.append(str(raw))

            if (str(type(item)) == "<class '__main__.MoveJ'>") or (str(type(item)) == "<class '__main__.MoveAbsJ'>") or (str(type(item)) == "<class '__main__.MoveL'>"):
                self.pts.append(item.get_rh_point3d(self.robot_root))
                self.quaternions.append(item.get_rh_quaternion(
                    self.robot_root))
                self.tools.append(item.tool)
                self.track_pos.append(item.get_eax_a())
                if str(type(item)) == "<class '__main__.MoveL'>":
                    self.move_types.append('moveL')
                elif (str(type(item)) == "<class '__main__.MoveJ'>") or (str(type(item)) == "<class '__main__.MoveAbsJ'>"):
                    self.move_types.append('moveJ')

        self.move_types.append('moveJ')
        self.toolpath = rg.PolylineCurve(self.pts)
        return self.toolpath


    def get_tool_plane(self, t):
        pt = self.toolpath.PointAtNormalizedLength(t)
        # Choose the segment by traveled length, not the nearest spatial point.
        # This avoids jumping across overlapping sections of the trajectory.
        _, crv_parameter = self.toolpath.LengthParameter(
            self.toolpath.GetLength() * max(0.0, min(1.0, float(t))))
        crv_idx = int(crv_parameter)
        return self.tools[crv_idx].tframe.to_rh_plane()


    def interpolate_moveL(self, crv_parameter, pt):
        crv_idx = int(crv_parameter)

        robot_root_int = self.robot_root.Clone()
        if crv_idx == len(self.track_pos) - 1:
            track_pos_int = self.track_pos[crv_idx]
        else:
            track_pos_int = lerp(self.track_pos[crv_idx],
                                 self.track_pos[crv_idx+1],
                                 crv_parameter - crv_idx)
        translate_int = -track_pos_int * robot_root_int.YAxis
        robot_root_int.Translate(translate_int)

        if crv_idx == len(self.quaternions) - 1:
            rotation = self.quaternions[crv_idx]
        else:
            rotation = quaternion_slerp(self.quaternions[crv_idx],
                                        self.quaternions[crv_idx+1],
                                        crv_parameter - crv_idx)

        plane = plane_from_pt_q(pt, rotation)
        tool_tcp = self.tools[crv_idx].tframe.to_rh_plane()

        flange_plane = get_flange_root(plane, robot_root_int, tool_tcp)
        rob_target = RobTarget.from_rh_plane(flange_plane)

        ik_solution = ik(rob_target)

        return ik_solution

    def interpolate_moveJ(self, crv_parameter):
        crv_idx = int(crv_parameter)
        tool_tcp = self.tools[crv_idx].tframe.to_rh_plane()

        robot_root_a = self.robot_root.Clone()
        track_pos_a = self.track_pos[crv_idx]
        translate_a = -track_pos_a * robot_root_a.YAxis
        robot_root_a.Translate(translate_a)

        plane_a = plane_from_pt_q(self.pts[crv_idx], self.quaternions[crv_idx])
        flange_plane_a = get_flange_root(plane_a, robot_root_a, tool_tcp)
        rob_target_a = RobTarget.from_rh_plane(flange_plane_a)
        ik_a = ik(rob_target_a)

        if crv_idx == len(self.pts) - 1:
            return ik_a

        robot_root_b = self.robot_root.Clone()
        track_pos_b = self.track_pos[crv_idx+1]
        translate_b = -track_pos_b * robot_root_b.YAxis
        robot_root_b.Translate(translate_b)

        plane_b = plane_from_pt_q(self.pts[crv_idx+1], self.quaternions[crv_idx+1])
        flange_plane_b = get_flange_root(plane_b, robot_root_b, tool_tcp)
        rob_target_b = RobTarget.from_rh_plane(flange_plane_b)
        ik_b = ik(rob_target_b)

        ik_pt = []
        t = crv_parameter - crv_idx
        print(t)
        for i in range(len(ik_a)):
            ik_pt.append(lerp(ik_a[i], ik_b[i], t))

        return ik_pt

    def interpolate(self, t):
        pt = self.toolpath.PointAtNormalizedLength(t)
        # Choose the segment by traveled length, not the nearest spatial point.
        # This avoids jumping across overlapping sections of the trajectory.
        _, crv_parameter = self.toolpath.LengthParameter(
            self.toolpath.GetLength() * max(0.0, min(1.0, float(t))))
        crv_idx = int(crv_parameter)

        robot_root_int = self.robot_root.Clone()
        if crv_idx == len(self.track_pos) - 1:
            track_pos_int = self.track_pos[crv_idx]
        else:
            track_pos_int = lerp(self.track_pos[crv_idx],
                                 self.track_pos[crv_idx+1],
                                 crv_parameter - crv_idx)
        translate_int = -track_pos_int * robot_root_int.YAxis
        robot_root_int.Translate(translate_int)

        if self.move_types[crv_idx+1] == 'moveL':
            return self.interpolate_moveL(crv_parameter, pt), robot_root_int

        elif self.move_types[crv_idx+1] == 'moveJ':
            return self.interpolate_moveJ(crv_parameter), robot_root_int
        


def transform_robot(robot_geo, robot_root, transforms, tool_geo=None):
    geo = th.tree_to_list(robot_geo)
    for i, link in enumerate(geo):
        transform = (rg.Transform.ChangeBasis(robot_root, rg.Plane.WorldXY)
                     * transforms[i])
        for part in link:
            part.Transform(transform)
        if (i == len(geo) - 1) and tool_geo:
            tool_geo.Transform(transform)
            geo.append(tool_geo)
            break

    return th.list_to_tree(geo, source=[0])




def tcp_to_brick_frame(tcp_frame):
    """Convert target TCP frame to brick-centred frame.

    Target TCP = brick frame rotated pi about the brick's local X axis.
    Rx(pi) is self-inverse, so the inverse mapping is also Rx(pi).
    This affects brick visualization only, not robot motion/commands.
    """
    if tcp_frame is None:
        return None
    brick_frame = rg.Plane(tcp_frame)
    brick_frame.Rotate(pi, brick_frame.XAxis)
    return brick_frame



def get_brick_display_state(toolpath, time_parameter, tcp_plane):
    """Read brick state from ordered events along path distance, not ClosestPoint.

    This affects only the three Grasshopper brick frame outputs.  No robot
    kinematics, RAPID commands, track position or motion interpolation changes.
    Convention from the current IRB4600 visualization script:
        Reset LL_GRIPPER -> pick up brick from jig
        Set LL_GRIPPER   -> release/place brick into wall
    """
    # The animation slider is normalized by toolpath length, NOT by instruction
    # count.  Recording an event at the last completed motion target places it
    # at the corresponding arclength, even when the path revisits that point.
    distances = [0.0]
    for i in range(1, len(toolpath.pts)):
        segment_length = toolpath.pts[i - 1].DistanceTo(toolpath.pts[i])
        distances.append(distances[-1] + segment_length)

    path_length = distances[-1] if distances else 0.0
    t = max(0.0, min(1.0, float(time_parameter)))
    traveled = path_length * t
    tolerance = max(1e-9, path_length * 1e-10)

    picked = 0
    placed = 0
    holding = False
    for motion_index, is_set_command in toolpath.brick_events:
        # An event after a move happens when that move's endpoint is reached,
        # not when the next move begins.  Preserve order for co-located events.
        if distances[motion_index] > traveled + tolerance:
            break
        if is_set_command:
            placed += 1
            holding = False
        else:
            picked += 1
            holding = True

    # These locations were recorded by the original code:
    # Reset commands -> place_locations (jig pickup sites)
    # Set commands   -> pickup_locations (wall placement sites)
    # Keep all jig frames until pickup, and add wall frames only after placement.
    jig_frames = [tcp_to_brick_frame(frame)
                  for _, frame in toolpath.place_locations[picked:]]
    wall_frames = [tcp_to_brick_frame(frame)
                   for _, frame in toolpath.pickup_locations[:placed]]
    held_frame = tcp_to_brick_frame(tcp_plane) if holding else None

    return held_frame, jig_frames, wall_frames, picked, placed, holding


def main():
    global robot_root, robot_geo, tool_geo
    global instructions, time_parameter
    global toolpath_crv, plane, transforms, geo
    global gripped_brick, jig_bricks, wall_bricks, current_brick_index

    if robot_root is None:
        robot_root = rg.Plane.WorldXY
    if instructions is None:
        instructions = []
    if time_parameter is None:
        time_parameter = 0.0

    toolpath = Toolpath(instructions, robot_root)
    toolpath_crv = toolpath.toolpath
    ik_solution, track_plane = toolpath.interpolate(time_parameter)
    transforms = fk(ik_solution)

    plane = get_tcp_plane_world(transforms[-1], track_plane,
                                   toolpath.get_tool_plane(time_parameter))

    geo = transform_robot(robot_geo, track_plane, transforms, tool_geo)

    # Brick display only: derive states from the chronological command timeline.
    # Never use ClosestPoint() or crv_idx+1 to decide when a brick changes state.
    (gripped_brick, jig_bricks, wall_bricks,
     n_picked, n_placed, holding) = get_brick_display_state(
         toolpath, time_parameter, plane)

    # Zero-based index of the brick being processed. Before pickup, this is
    # the next brick in the jig; while holding, it is the carried brick.
    # Return None after the final brick has been placed.
    total_bricks = len(toolpath.place_locations)
    if holding and n_picked > 0:
        current_brick_index = n_picked - 1
    elif n_placed < total_bricks:
        current_brick_index = n_placed
    else:
        current_brick_index = None

    print("Brick state: t={:.5f}, Reset/pickup={}/{}, Set/placed={}/{}, "
          "holding={}, jig={}, wall={}".format(
              float(time_parameter), n_picked, toolpath.release_commands,
              n_placed, toolpath.grip_commands,
              holding, len(jig_bricks), len(wall_bricks)))
    if not toolpath.grip_commands and not toolpath.release_commands:
        print("Non-motion instruction samples:", toolpath.brick_command_samples)


if __name__ == "__main__":
    main()
