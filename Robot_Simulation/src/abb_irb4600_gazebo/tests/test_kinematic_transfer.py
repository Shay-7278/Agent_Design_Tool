"""Independent URDF-chain checks, including all flange axes, not DH against itself."""
import json
import random
import unittest
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
import xacro
from ament_index_python.packages import get_package_share_directory as share
from abb_irb4600_gazebo.irb4600_kinematics import forward_kinematics_urdf, solve_base_target
from abb_irb4600_gazebo.motion import Pose
from abb_irb4600_gazebo.target_motion import brick_frame_from_tcp, tcp_target_from_brick


class KinematicTransferTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = ET.fromstring(xacro.process_file(share('abb_irb4600_workcell_description')+'/urdf/abb_irb4600_workcell.xacro').toxml())
        cls.joints = [cls.robot.find(f"./joint[@name='joint_LL_{i}']") for i in range(1,7)]
        cls.limits = [tuple(float(j.find('limit').get(k)) for k in ['lower','upper']) for j in cls.joints]

    def urdf_fk(self, joints):
        t = np.eye(4)
        for j,q in zip(self.joints,joints):
            origin=j.find('origin');m=np.eye(4)
            m[:3,3]=list(map(float,origin.get('xyz').split()))
            m[:3,:3]=Rotation.from_euler('xyz',list(map(float,origin.get('rpy').split()))).as_matrix()
            r=np.eye(4);axis=np.array(list(map(float,j.find('axis').get('xyz').split())))
            r[:3,:3]=Rotation.from_rotvec(axis*q).as_matrix()
            t=t@m@r
        return t

    def test_gh_to_urdf_adapter_matches_independent_link_chain_all_axes(self):
        rng=random.Random(574)
        for q in [[0]*6]+[[rng.uniform(-.7,.7) for _ in range(6)] for _ in range(30)]:
            np.testing.assert_allclose(forward_kinematics_urdf(q),self.urdf_fk(q),atol=1e-10)

    def test_tcp_targets_roundtrip_through_actual_urdf(self):
        for xyz in [(1.6018,-.2439,.307),(1.6018,-.2439,.557)]:
            for yaw in [-.8,0,.8]:
                target=Pose(xyz,tuple(Rotation.from_euler('xyz',[np.pi,0,yaw]).as_quat()))
                q=solve_base_target(target,self.limits)
                actual=self.urdf_fk(q)
                np.testing.assert_allclose(actual[:3,3],xyz,atol=1e-7)
                np.testing.assert_allclose(actual[:3,:3],Rotation.from_quat(target.orientation_xyzw).as_matrix(),atol=1e-7)

    def test_verified_gh_home_and_speed_profile(self):
        profile=json.loads((Path(share('abb_irb4600_gazebo'))/'config/gh_motion_profile.json').read_text())
        self.assertEqual(profile['retract_m'],.2)
        self.assertEqual(profile['track_offset_m'],-1)
        self.assertEqual(profile['move_j_speed_m_s'],.5)
        self.assertEqual(profile['move_l_speed_m_s'],.05)
        h=profile['home_tcp'];tcp=Pose(tuple(h['position']),tuple(h['orientation_xyzw']))
        converted=tcp_target_from_brick(brick_frame_from_tcp(tcp))
        np.testing.assert_allclose(converted.orientation_xyzw,tcp.orientation_xyzw,atol=1e-12)
        np.testing.assert_allclose(tcp.position,(.5,3.45103,1))
