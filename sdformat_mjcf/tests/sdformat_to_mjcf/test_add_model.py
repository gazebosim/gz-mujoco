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
from gz.math import Pose3d
from dm_control import mjcf

from sdformat_mjcf.sdformat_to_mjcf.converters.model import add_model
from sdformat_mjcf.sdformat_to_mjcf.converters.root import add_root
import sdformat_mjcf.utils.sdf_utils as su
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
        joint.set_parent_name("base_link")
        joint.set_child_name("upper_link")
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


class ModelIntegrationTest(unittest.TestCase):
    def test_static_model(self):
        test_model_sdf = """
        <sdf version="1.6">
            <model name="test_model">
                <static>true</static>
                <link name="link1">
                    <collision name="c1">
                        <geometry>
                            <sphere><radius>2</radius></sphere>
                        </geometry>
                    </collision>
                </link>
            </model>
        </sdf>
        """
        root = sdf.Root()
        root.load_sdf_string(test_model_sdf)
        mj_root = add_root(root)
        self.assertIsNotNone(mj_root)
        mj_link1 = mj_root.find("body", "link1")
        self.assertFalse(mj_link1.get_children("joint"))
        self.assertFalse(mj_link1.get_children("freejoint"))

    def test_nested_model_fixed_joint(self):
        test_model_sdf = """
        <sdf version="1.11">
            <model name="modelA">
                <pose>1 0 0 0 0 0</pose>
                <link name="linkA">
                    <pose>1 0 0 0 0 0</pose>
                    <collision name="c1">
                        <geometry>
                            <sphere><radius>1</radius></sphere>
                        </geometry>
                    </collision>
                </link>
                <model name="modelB">
                    <pose>0 1 0 0 0 0</pose>
                    <link name="linkA">
                        <pose>0 1 0 0 0 0</pose>
                        <collision name="c1">
                            <geometry>
                                <sphere><radius>2</radius></sphere>
                            </geometry>
                        </collision>
                    </link>
                </model>
                <joint name="modelB_joint" type="fixed">
                    <parent>linkA</parent>
                    <child>modelB::linkA</child>
                </joint>
            </model>
        </sdf>
        """
        root = sdf.Root()
        root.load_sdf_string(test_model_sdf)
        mj_root = add_root(root)
        self.assertIsNotNone(mj_root)
        mj_modelA_linkA = mj_root.find("body", "linkA")
        self.assertIsNotNone(mj_modelA_linkA)

        # Check that modelB::linkA was added as a child of (modelA::)linkA
        modelA_linkA_children = mj_modelA_linkA.get_children("body")
        self.assertEqual(1, len(modelA_linkA_children))
        mj_modelB_linkA = modelA_linkA_children[0]
        self.assertEqual("modelB::linkA", mj_modelB_linkA.name)

        # Verify that the right pos was set for modelB::linkA
        sdf_linkA = root.model().link_by_name("linkA")
        sdf_linkA_pose = sdf_linkA.semantic_pose().resolve(
            _resolveTo="modelB::linkA")
        modelB_linkA_expected_pos = su.vec3d_to_list(
            sdf_linkA_pose.inverse().pos())
        assert_allclose(modelB_linkA_expected_pos, mj_modelB_linkA.pos)

    def test_nested_model_multi_level(self):
        test_model_sdf = """
        <sdf version="1.11">
            <model name='top_nested'>
                <model name='sub_nested'>
                    <model name='simple_model'>
                        <link name='link'/>
                    </model>
                    <model name='extra_model'>
                        <link name='link'/>
                    </model>
                </model>
            </model>
        </sdf>
        """
        root = sdf.Root()
        root.load_sdf_string(test_model_sdf)
        mj_root = add_root(root)
        self.assertIsNotNone(mj_root)
        mj_simple_model_link = mj_root.find("body",
                                            "sub_nested::simple_model::link")
        self.assertIsNotNone(mj_simple_model_link)
        self.assertIsNotNone(mj_simple_model_link.get_children("freejoint"))

        mj_extra_model_link = mj_root.find("body",
                                           "sub_nested::extra_model::link")
        self.assertIsNotNone(mj_extra_model_link)
        self.assertIsNotNone(mj_extra_model_link.get_children("freejoint"))

    def test_nested_model_fixed_joint_with_nested_parent_link(self):
        test_model_sdf = """
        <sdf version="1.11">
            <model name="modelA">
                <pose>1 0 0 0 0 0</pose>
                <link name="linkA">
                    <pose>1 0 0 0 0 0</pose>
                    <collision name="c1">
                        <geometry>
                            <sphere><radius>1</radius></sphere>
                        </geometry>
                    </collision>
                </link>
                <model name="modelB">
                    <pose>0 1 0 0 0 0</pose>
                    <link name="linkA">
                        <pose>0 10 0 0 0 0</pose>
                        <collision name="c2">
                            <geometry>
                                <sphere><radius>2</radius></sphere>
                            </geometry>
                        </collision>
                    </link>
                </model>

                <joint name="modelB_joint" type="fixed">
                    <parent>modelB::linkA</parent>
                    <child>linkA</child>
                </joint>
            </model>
        </sdf>
        """
        root = sdf.Root()
        root.load_sdf_string(test_model_sdf)
        mj_root = add_root(root)
        self.assertIsNotNone(mj_root)

        mj_modelB_linkA = mj_root.find("body", "modelB::linkA")
        self.assertIsNotNone(mj_modelB_linkA)

        # Check that (modelA::)linkA was added as a child of modelB::linkA
        modelB_linkA_children = mj_modelB_linkA.get_children("body")
        self.assertEqual(1, len(modelB_linkA_children))
        mj_modelA_linkA = modelB_linkA_children[0]
        self.assertEqual("linkA", mj_modelA_linkA.name)

        sdf_modelA = root.model()

        # Verify that the right pos was set for top level node modelB::linkA
        sdf_modelB_linkA_pose = sdf_modelA.semantic_pose().resolve(
            _resolveTo="modelA::modelB::linkA").inverse()
        modelB_linkA_expected_pos = su.vec3d_to_list(
            (sdf_modelA.raw_pose() * sdf_modelB_linkA_pose).pos()
        )
        assert_allclose(modelB_linkA_expected_pos, mj_modelB_linkA.pos)

        # Verify that the right pos was set for (modelA::)linkA
        sdf_linkA = sdf_modelA.link_by_name("linkA")
        sdf_linkA_pose = sdf_linkA.semantic_pose().resolve(
            _resolveTo="modelB::linkA")
        modelA_linkA_expected_pos = su.vec3d_to_list(sdf_linkA_pose.pos())
        assert_allclose(modelA_linkA_expected_pos, mj_modelA_linkA.pos)


if __name__ == "__main__":
    unittest.main()
