"""Cached basis changes must preserve moving-rail and nonidentity-tool results."""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch
import numpy as np
from geometry_msgs.msg import TransformStamped
from abb_irb4600_gazebo.execute_one_brick import OneBrickExecutor, _qrot, _transform_pose
from abb_irb4600_gazebo.motion import Pose, rail_position
from abb_irb4600_gazebo.target_motion import rpy_to_quaternion, tcp_target_from_brick, flange_target_from_tcp


def transform(xyz, rpy):
    tf = TransformStamped()
    t, q = tf.transform.translation, tf.transform.rotation
    t.x, t.y, t.z = xyz
    q.x, q.y, q.z, q.w = rpy_to_quaternion(*rpy)
    return tf


class FixedTransformCacheTests(unittest.TestCase):
    def test_cached_solver_matches_previous_basis_math_across_rail_positions(self):
        base = transform((1.2, -.5, -.017), (.001, -.002, -1.57))
        track = transform((3.69928, 3.45103, .0175951), (.0008572, -.0012691, -3.1417))
        bq = base.transform.rotation; tq = track.transform.rotation
        direction = _qrot((tq.x,tq.y,tq.z,tq.w), (1,0,0))
        origin = (track.transform.translation.x,track.transform.translation.y,track.transform.translation.z)
        tool = Pose((.01,-.02,.24313),rpy_to_quaternion(.05,-.1,.03))
        for reference_rail in (0, 1.8, 3.89):
            node = OneBrickExecutor.__new__(OneBrickExecutor)
            lookup = Mock(return_value=track)
            node.tf_buffer = SimpleNamespace(lookup_transform=lookup)
            node.base_tf, node.flange_tcp = base, tool
            node.reference_rail, node.profile = reference_rail, {'track_offset_m': -1, 'cfx': 0}
            node.limits = [(-6,6)]*6
            node._cache_fixed_transforms()
            lookup.side_effect = AssertionError('planning queried a fixed TF again')
            for x in (-.5, .5, 2.5, 4.5):
                for yaw in (-.8, .3):
                    target = Pose((x,2.7,.4),rpy_to_quaternion(0,0,yaw))
                    rail = rail_position(target,origin,direction,offset_m=-1,limits_m=(0,3.89))
                    root = _transform_pose(target,base)
                    delta = _qrot((bq.x,bq.y,bq.z,bq.w),tuple(v*(reference_rail-rail) for v in direction))
                    root = Pose(tuple(v+d for v,d in zip(root.position,delta)),root.orientation_xyzw)
                    expected = flange_target_from_tcp(tcp_target_from_brick(root),tool)
                    with patch('abb_irb4600_gazebo.execute_one_brick.solve_base_target', return_value=(0,)*6) as solve:
                        actual_rail, _ = node._solve(target,(0,)*6)
                    actual = solve.call_args.args[0]
                    self.assertAlmostEqual(actual_rail,rail)
                    np.testing.assert_allclose(actual.position,expected.position,atol=1e-12)
                    np.testing.assert_allclose(actual.orientation_xyzw,expected.orientation_xyzw,atol=1e-12)
