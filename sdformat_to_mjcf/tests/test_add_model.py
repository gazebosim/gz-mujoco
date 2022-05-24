# Copyright (C) 2022 Open Source Robotics Foundation
#
# Licensed under the Apache License, version 2.0 (the "License")
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import unittest
from numpy.testing import assert_allclose
from math import pi

import sdformat as sdf
from ignition.math import Pose3d
from dm_control import mjcf

from sdformat_to_mjcf.converters.model import add_model
import sdformat_mjcf_utils.sdf_utils as su
from tests import helpers


class ModelTest(helpers.TestCase):
    test_pose = Pose3d(1, 2, 3, pi / 2, pi / 3, pi / 4)
    expected_pos = [1.0, 2.0, 3.0]
    expected_euler = [90.0, 60.0, 45.0]

    def setUp(self):
        self.mujoco = mjcf.RootElement(model="test")

    def test_basic_model(self):
        model = sdf.Model()
        model.set_name("test_model")
        model.set_raw_pose(self.test_pose)

        base_link = sdf.Link()
        base_link.set_name("base_link")
        model.add_link(base_link)

        mj_root = add_model(self.mujoco, model)
        self.assertIsNotNone(mj_root)
        mj_bodies = mj_root.worldbody.get_children("body")
        self.assertEqual(1, len(mj_bodies))
        self.assertEqual("base_link", mj_bodies[0].name)

        assert_allclose(self.expected_pos, mj_bodies[0].pos)
        assert_allclose(self.expected_euler, mj_bodies[0].euler)

    def test_model_multiple_floating_links(self):
        model_raw_pose = self.test_pose
        model = sdf.Model()
        model.set_name("test_model")
        model.set_raw_pose(model_raw_pose)

        float_link_1 = sdf.Link()
        float_link_1.set_name("float_link_1")
        model.add_link(float_link_1)

        float_link_2 = sdf.Link()
        float_link_2.set_name("float_link_2")
        float_link_2_raw_pose = Pose3d(-0.1, -0.2, -0.3, 0, 0, 0)
        float_link_2.set_raw_pose(float_link_2_raw_pose)
        model.add_link(float_link_2)

        mj_root = add_model(self.mujoco, model)
        self.assertIsNotNone(mj_root)

        mj_float_link_1 = mj_root.worldbody.find('body', 'float_link_1')
        self.assertIsNotNone(mj_float_link_1)

        mj_float_link_2 = mj_root.worldbody.find('body', 'float_link_2')
        self.assertIsNotNone(mj_float_link_2)

        float_link_1_expected_pos = su.vec3d_to_list(model_raw_pose.pos())
        float_link_1_expected_euler = su.quat_to_euler_list(
            model_raw_pose.rot())
        assert_allclose(float_link_1_expected_pos, mj_float_link_1.pos)
        assert_allclose(float_link_1_expected_euler, mj_float_link_1.euler)

        float_link_2_expected_pose = model_raw_pose * float_link_2_raw_pose
        float_link_2_expected_pos = su.vec3d_to_list(
            float_link_2_expected_pose.pos())
        float_link_2_expected_euler = su.quat_to_euler_list(
            float_link_2_expected_pose.rot())
        assert_allclose(float_link_2_expected_pos, mj_float_link_2.pos)
        assert_allclose(float_link_2_expected_euler, mj_float_link_2.euler)

    def test_model_multiple_links_with_joint(self):
        model_raw_pose = self.test_pose
        model = sdf.Model()
        model.set_name("test_model")
        model.set_raw_pose(model_raw_pose)

        base_link = sdf.Link()
        base_link.set_name("base_link")
        model.add_link(base_link)

        upper_link = sdf.Link()
        upper_link.set_name("upper_link")
        upper_link_raw_pose = Pose3d(-0.1, -0.2, -0.3, 0, 0, 0)
        upper_link.set_raw_pose(upper_link_raw_pose)
        model.add_link(upper_link)

        joint = sdf.Joint()
        joint.set_name("joint")
        joint.set_type(sdf.JointType.FIXED)
        joint.set_parent_link_name("base_link")
        joint.set_child_link_name("upper_link")
        model.add_joint(joint)

        mj_root = add_model(self.mujoco, model)
        self.assertIsNotNone(mj_root)

        mj_base_link = mj_root.worldbody.find('body', 'base_link')
        self.assertIsNotNone(mj_base_link)

        mj_upper_link = mj_root.worldbody.find('body', 'upper_link')
        self.assertIsNotNone(mj_upper_link)

        base_link_expected_pos = su.vec3d_to_list(model_raw_pose.pos())
        base_link_expected_euler = su.quat_to_euler_list(model_raw_pose.rot())
        assert_allclose(base_link_expected_pos, mj_base_link.pos)
        assert_allclose(base_link_expected_euler, mj_base_link.euler)

        # upper_link has a pose `upper_link_raw_pose` relative to the model
        # frame. If it a floating body under worldbody, it's pose would be:
        # `model_raw_pose * upper_link_raw_pose`. However, since it's a child
        # of `base_link`, and since `base_link` has identity pose, the pose of
        # `upper_link` relative to `base_link` is just `upper_link_raw_pose`.
        # i.e., `model_raw_pose` does not affect the poser of upper_link
        upper_link_expected_pos = su.vec3d_to_list(upper_link_raw_pose.pos())
        upper_link_expected_euler = su.quat_to_euler_list(
            upper_link_raw_pose.rot())
        assert_allclose(upper_link_expected_pos, mj_upper_link.pos)
        assert_allclose(upper_link_expected_euler, mj_upper_link.euler)

    def test_collision_exclusions(self):
        test_model_sdf = """
        <sdf version="1.6">
            <model name="test_model">
                <joint name="joint_to_world" type="fixed">
                    <parent>world</parent>
                    <child>link1</child>
                </joint>
                <link name="link1"/>
                <joint name="joint1" type="revolute">
                    <parent>link1</parent>
                    <child>link2</child>
                    <axis><xyz>1 0 0</xyz></axis>
                </joint>
                <link name="link2"/>
            </model>
        </sdf>
        """

        root = sdf.Root()
        root.load_sdf_string(test_model_sdf)
        mj_root = add_model(self.mujoco, root.model())
        self.assertIsNotNone(mj_root)
        excludes = mj_root.contact.get_children("exclude")
        self.assertEqual(1, len(excludes))
        self.assertEqual("link1_link2", excludes[0].name)
        self.assertEqual("link1", excludes[0].body1)
        self.assertEqual("link2", excludes[0].body2)


if __name__ == "__main__":
    unittest.main()
