#!/usr/bin/env python

import re
from math import ceil, sqrt
import Rhino.Geometry as rg


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


def main():
    global tool_frame, tool_load, name, tool

    if not isinstance(tool_frame, rg.Plane):
        raise TypeError("Tool frame not a valid plane!")

    if tool_load is None:
        tool_load = LoadData()
        tool_load.mass = 1
        tool_load.cog.z = 20

    if name is None:
        name = "Tool0"

    tframe = Pose.from_rh_plane(tool_frame)
    tool = ToolData(True, tframe, tool_load, name)

    return


if __name__ == "__main__":
    main()
