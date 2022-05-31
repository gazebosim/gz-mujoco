# Copyright (C) 2022 Open Source Robotics Foundation

# Licensed under the Apache License, version 2.0 (the "License")
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#       http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import unittest
import numpy as np
from numpy.testing import assert_allclose
from math import pi

import sdformat as sdf
from ignition.math import Pose3d, Vector3d
from dm_control import mjcf

from sdformat_to_mjcf.converters.joint import add_joint
from sdformat_to_mjcf.sdf_kinematics import FreeJoint, StaticFixedJoint
import sdformat_mjcf_utils.sdf_utils as su
from tests import helpers


class JointTest(helpers.TestCase):

    test_pose = Pose3d(1, 2, 3, pi / 2, pi / 3, pi / 4)
    expected_pos = [1.0, 2.0, 3.0]
    expected_euler = [90.0, 60.0, 45.0]

    def setUp(self):
        self.mujoco = mjcf.RootElement(model="test")
        self.body = self.mujoco.worldbody.add("body", name="base_link")

    def create_sdf_joint(self,
                         name,
                         j_type,
                         pose,
                         xyz=None,
                         limits=None,
                         dynamics=None):
        joint = sdf.Joint()
        joint.set_name(name)
        joint.set_type(j_type)
        joint.set_raw_pose(pose)
        joint_axis = sdf.JointAxis()
        if xyz:
            joint_axis.set_xyz(Vector3d(*xyz))
        if limits:
            joint_axis.set_lower(limits[0])
            joint_axis.set_upper(limits[1])

        if dynamics:
            for key, val in dynamics.items():
                setter = getattr(joint_axis, f"set_{key}")
                setter(val)

        joint.set_axis(0, joint_axis)
        return joint

    def test_free_joint(self):
        mj_joint = add_joint(self.body, FreeJoint())
        self.assertIsNotNone(mj_joint)
        self.assertEqual("freejoint", mj_joint.tag)

    def test_multiple_free_joints(self):
        mj_joint1 = add_joint(self.body, FreeJoint())
        self.assertIsNotNone(mj_joint1)
        # Add another body with a free joint to worldbody
        body2 = self.mujoco.worldbody.add("body", name="test_body")
        mj_joint2 = add_joint(body2, FreeJoint())
        self.assertIsNotNone(mj_joint2)

    def test_fixed_joint(self):
        joint = sdf.Joint()
        joint.set_type(sdf.JointType.FIXED)
        mj_joint = add_joint(self.body, joint)
        self.assertIsNone(mj_joint)

    def test_static_fixed_joint(self):
        mj_joint = add_joint(self.body, StaticFixedJoint())
        self.assertIsNone(mj_joint)

    def test_revolute_joint(self):
        joint = self.create_sdf_joint("joint1",
                                      sdf.JointType.REVOLUTE,
                                      self.test_pose,
                                      xyz=[1, 0, 0])

        mj_joint = add_joint(self.body, joint)
        self.assertIsNotNone(mj_joint)
        self.assertEqual("joint1", mj_joint.name)
        self.assertEqual("hinge", mj_joint.type)
        assert_allclose(self.expected_pos, mj_joint.pos)
        expected_axis = su.vec3d_to_list(
            self.test_pose.rot().rotate_vector(Vector3d(1, 0, 0))
        )
        assert_allclose(expected_axis, mj_joint.axis)
        self.assertFalse(mj_joint.limited)

    def test_revolute_joint_with_limits(self):
        joint = self.create_sdf_joint("joint1",
                                      sdf.JointType.REVOLUTE,
                                      self.test_pose,
                                      xyz=[1, 0, 0],
                                      limits=(-pi / 4, pi / 2))

        mj_joint = add_joint(self.body, joint)
        self.assertTrue(mj_joint.limited)
        assert_allclose([-45.0, 90.0], mj_joint.range)

    def test_revolute_joint_with_dynamics(self):
        joint_dynamics = {
            'damping': 0.1,
            'friction': 0.2,
            'spring_stiffness': 0.3,
            'spring_reference': pi / 6
        }
        joint = self.create_sdf_joint("joint1",
                                      sdf.JointType.REVOLUTE,
                                      self.test_pose,
                                      xyz=[1, 0, 0],
                                      dynamics=joint_dynamics)

        mj_joint = add_joint(self.body, joint)
        self.assertEqual("hinge", mj_joint.type)
        self.assertEqual(0.1, mj_joint.damping)
        self.assertEqual(0.2, mj_joint.frictionloss)
        self.assertEqual(0.3, mj_joint.stiffness)
        self.assertAlmostEqual(30, mj_joint.springref)

    def test_continuous_joint(self):
        joint = self.create_sdf_joint("joint1",
                                      sdf.JointType.CONTINUOUS,
                                      self.test_pose,
                                      xyz=[1, 0, 0],
                                      limits=(-pi / 4, pi / 2))

        mj_joint = add_joint(self.body, joint)
        self.assertIsNotNone(mj_joint)
        self.assertEqual("joint1", mj_joint.name)
        self.assertEqual("hinge", mj_joint.type)
        assert_allclose(self.expected_pos, mj_joint.pos)
        expected_axis = su.vec3d_to_list(
            self.test_pose.rot().rotate_vector(Vector3d(1, 0, 0))
        )
        assert_allclose(expected_axis, mj_joint.axis)
        # Limits should be ignored even if they're set in the SDFormat Joint
        self.assertFalse(mj_joint.limited)

    def test_continuous_joint_with_dynamics(self):
        joint_dynamics = {
            'damping': 0.1,
            'friction': 0.2,
            'spring_stiffness': 0.3,
            'spring_reference': pi / 6
        }
        joint = self.create_sdf_joint("joint1",
                                      sdf.JointType.CONTINUOUS,
                                      self.test_pose,
                                      xyz=[1, 0, 0],
                                      dynamics=joint_dynamics)

        mj_joint = add_joint(self.body, joint)
        self.assertEqual("hinge", mj_joint.type)
        self.assertEqual(0.1, mj_joint.damping)
        self.assertEqual(0.2, mj_joint.frictionloss)
        self.assertEqual(0.3, mj_joint.stiffness)
        self.assertAlmostEqual(30, mj_joint.springref)

    def test_prismatic_joint(self):
        joint = self.create_sdf_joint("joint1",
                                      sdf.JointType.PRISMATIC,
                                      self.test_pose,
                                      xyz=[1, 0, 0])

        mj_joint = add_joint(self.body, joint)
        self.assertIsNotNone(mj_joint)
        self.assertEqual("joint1", mj_joint.name)
        self.assertEqual("slide", mj_joint.type)
        assert_allclose(self.expected_pos, mj_joint.pos)
        expected_axis = su.vec3d_to_list(
            self.test_pose.rot().rotate_vector(Vector3d(1, 0, 0))
        )
        assert_allclose(expected_axis, mj_joint.axis)

    def test_prismatic_joint_with_limits(self):
        joint = self.create_sdf_joint("joint1",
                                      sdf.JointType.PRISMATIC,
                                      self.test_pose,
                                      xyz=[1, 0, 0],
                                      limits=(-5, 10))

        mj_joint = add_joint(self.body, joint)
        self.assertTrue(mj_joint.limited)
        assert_allclose([-5, 10], mj_joint.range)

    def test_prismatic_joint_with_dynamics(self):
        joint_dynamics = {
            'damping': 0.1,
            'friction': 0.2,
            'spring_stiffness': 0.3,
            'spring_reference': 0.4
        }
        joint = self.create_sdf_joint("joint1",
                                      sdf.JointType.PRISMATIC,
                                      self.test_pose,
                                      xyz=[1, 0, 0],
                                      dynamics=joint_dynamics)

        mj_joint = add_joint(self.body, joint)
        self.assertEqual(0.1, mj_joint.damping)
        self.assertEqual(0.2, mj_joint.frictionloss)
        self.assertEqual(0.3, mj_joint.stiffness)
        self.assertAlmostEqual(0.4, mj_joint.springref)

    def test_ball_joint(self):
        joint = self.create_sdf_joint("joint1",
                                      sdf.JointType.BALL,
                                      self.test_pose)

        mj_joint = add_joint(self.body, joint)
        self.assertIsNotNone(mj_joint)
        self.assertEqual("joint1", mj_joint.name)
        self.assertEqual("ball", mj_joint.type)
        assert_allclose(self.expected_pos, mj_joint.pos)

    def test_force_torque_sensor(self):
        joint = self.create_sdf_joint("joint1", sdf.JointType.FIXED,
                                      self.test_pose)
        sensor = sdf.Sensor()
        sensor.set_name("ft_sensor")
        sensor.set_type(sdf.Sensortype.FORCE_TORQUE)
        sensor.set_raw_pose(self.test_pose)
        ft_sensor = sdf.ForceTorque()
        sensor.set_force_torque_sensor(ft_sensor)
        joint.add_sensor(sensor)

        add_joint(self.body, joint)

        self.assertIsNotNone(self.mujoco.sensor)
        mjcf_force_sensors = self.mujoco.sensor.get_children("force")
        self.assertEqual(1, len(mjcf_force_sensors))
        mjcf_torque_sensors = self.mujoco.sensor.get_children("torque")
        self.assertEqual(1, len(mjcf_torque_sensors))

    def test_force_torque_sensor_frames(self):
        joint = self.create_sdf_joint("joint1", sdf.JointType.FIXED,
                                      self.test_pose)
        sensor = sdf.Sensor()
        sensor.set_name("ft_sensor")
        sensor.set_type(sdf.Sensortype.FORCE_TORQUE)

        sensor_pos = [0, 0, 1]
        sensor_euler = np.array([90, 0, 0])
        sensor.set_raw_pose(Pose3d(*sensor_pos, *(np.radians(sensor_euler))))

        parent_pos = [0, 2, 0]
        parent_euler = [0, 0, 45]
        child_pos = [1, 0, 0]
        child_euler = [0, 90, 0]

        joint_pos = self.expected_pos
        FTF = sdf.ForceTorqueFrame
        frames_with_expected_poses = (
            (FTF.PARENT, joint_pos, parent_euler),
            (FTF.CHILD, joint_pos, child_euler),
            (FTF.SENSOR, joint_pos, sensor_euler)
        )

        force_torque = sdf.ForceTorque()

        for frame, frame_pos, frame_euler in frames_with_expected_poses:
            self.setUp()

            self.body.pos = parent_pos
            self.body.euler = parent_euler
            child_body = self.body.add("body",
                                       name="child_body",
                                       pos=child_pos,
                                       euler=child_euler)
            joint.clear_sensors()

            force_torque.set_frame(frame)
            sensor.set_force_torque_sensor(force_torque)
            joint.add_sensor(sensor)
            add_joint(child_body, joint)

            self.assertIsNotNone(self.mujoco.sensor)
            mjcf_force_sensors = self.mujoco.sensor.get_children("force")
            self.assertEqual(1, len(mjcf_force_sensors))
            mjcf_torque_sensors = self.mujoco.sensor.get_children("torque")
            self.assertEqual(1, len(mjcf_torque_sensors))

            sensor_sites = child_body.get_children("site")
            self.assertEqual(1, len(sensor_sites))
            assert_allclose(frame_pos, sensor_sites[0].pos)
            assert_allclose(frame_euler, sensor_sites[0].euler)


if __name__ == "__main__":
    unittest.main()
