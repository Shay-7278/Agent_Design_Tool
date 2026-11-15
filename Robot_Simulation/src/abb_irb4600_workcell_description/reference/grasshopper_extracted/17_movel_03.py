#!/usr/bin/env python

import re
from math import ceil, sqrt, radians, cos, sin
import Rhino.Geometry as rg
import ghpythonlib.treehelpers as th


__author__ = "Daniel Ruan"
__copyright__ = "Copyright 2023, ADR Laboratory"
__credits__ = ["Daniel Ruan", "Arash Adel"]
__license__ = "MIT"
__email__ = "daniel.ruan@princeton.edu"


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
    flange_plane_world = tool_tcp.Clone()
    target_transform = rg.Transform.ChangeBasis(target_plane, rg.Plane.WorldXY)
    flange_plane_world.Transform(target_transform)

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
        return ik.ik(target)

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



def main():
    global target_pose, speed, zone, tool, instruction
    global track_offset, rob_root, rob_conf

    if not isinstance(target_pose, rg.Plane):
        raise TypeError("Target pose not a valid plane!")

    if speed is None:
        speed = "v10"

    if zone is None:
        zone = "fine"

    if tool is None:
        tool = ToolData(name="Tool0")

    if track_offset is None:
        track_offset = 0.0

    if rob_root is None:
        rob_root = rg.Plane.WorldXY

    if rob_conf is None:
        rob_conf = [0, 0, 0, 0]

    speed = SpeedData.predefined(speed)
    zone = ZoneData.predefined(zone)

    robtarget = RobTarget.from_rh_plane(target_pose)
    robtarget.set_track_offset(target_pose, track_offset, root=rob_root)
    robtarget.robconf = ConfData.from_list(rob_conf)
    instruction = MoveL(robtarget, speed, zone, tool)

    instruction = th.list_to_tree([instruction], source=[0])

    return


if __name__ == "__main__":
    main()
