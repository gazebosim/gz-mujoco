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
import sdformat as sdf
from tests import helpers

from dm_control import mjcf
from sdformat_to_mjcf.converters.geometry import add_collision
from sdformat_to_mjcf.converters.root import add_root


GeometryType = sdf.Geometry.GeometryType


class NameCollisionTest(helpers.TestCase):

    def create_geometry_container(self, ContainerType):
        container = ContainerType()
        container.set_name("bumper")

        geometry = sdf.Geometry()
        geometry.set_box_shape(sdf.Box())
        geometry.set_type(GeometryType.BOX)
        container.set_geometry(geometry)
        return container

    def test_collision(self):
        collision = self.create_geometry_container(sdf.Collision)
        mujoco = mjcf.RootElement(model="test")
        body1 = mujoco.worldbody.add("body", name="body1")
        mj_geom1 = add_collision(body1, collision)
        self.assertEqual("bumper", mj_geom1.name)

        body2 = mujoco.worldbody.add("body", name="body2")
        mj_geom2 = add_collision(body2, collision)
        self.assertEqual("bumper_0", mj_geom2.name)

    def test_visual(self):
        visual = self.create_geometry_container(sdf.Visual)

        mujoco = mjcf.RootElement(model="test")
        body1 = mujoco.worldbody.add("body", name="body1")
        mj_geom1 = add_collision(body1, visual)
        self.assertEqual("bumper", mj_geom1.name)

        body2 = mujoco.worldbody.add("body", name="body2")
        mj_geom2 = add_collision(body2, visual)
        self.assertEqual("bumper_0", mj_geom2.name)

    def test_link(self):
        sdf_string = """
        <sdf version="1.6">
            <world name="test_world">
                <model name="M1">
                    <link name="base_link"/>
                </model>
                <model name="M2">
                    <link name="base_link"/>
                </model>
                <model name="M3">
                    <link name="base_link"/>
                </model>
            </world>
        </sdf>
        """
        root = sdf.Root()
        errors = root.load_sdf_string(sdf_string)
        self.assertEqual(0, len(errors))
        mjcf_root = add_root(root)
        bodies = mjcf_root.find_all("body")
        self.assertEqual(3, len(bodies))
        name_possibilities = ["base_link", "base_link_0", "base_link_1"]
        for body in bodies:
            self.assertIn(body.name, name_possibilities)

    def test_freejoints(self):
        sdf_string = """
        <sdf version="1.6">
            <model name="M1">
                <link name="link1"/>
                <link name="link2"/>
                <link name="link3"/>
            </model>
        </sdf>
        """
        root = sdf.Root()
        errors = root.load_sdf_string(sdf_string)
        self.assertEqual(0, len(errors))
        mjcf_root = add_root(root)
        joints = mjcf_root.find_all("joint")
        self.assertEqual(3, len(joints))

    def test_joints(self):
        sdf_string = """
        <sdf version="1.6">
            <world name="default">
                <model name="M1">
                    <link name="link1"/>
                    <joint name="test_joint" type="revolute">
                        <parent>link1</parent>
                        <child>link2</child>
                        <axis><xyz>0 0 1</xyz></axis>
                    </joint>
                    <link name="link2"/>
                </model>
                <model name="M2">
                    <link name="link1"/>
                    <joint name="test_joint" type="revolute">
                        <parent>link1</parent>
                        <child>link2</child>
                        <axis><xyz>0 0 1</xyz></axis>
                    </joint>
                    <link name="link2"/>
                </model>
                <model name="M3">
                    <link name="link1"/>
                    <joint name="test_joint" type="revolute">
                        <parent>link1</parent>
                        <child>link2</child>
                        <axis><xyz>0 0 1</xyz></axis>
                    </joint>
                    <link name="link2"/>
                </model>
            </world>
        </sdf>
        """
        root = sdf.Root()
        errors = root.load_sdf_string(sdf_string)
        self.assertEqual(0, len(errors))
        mjcf_root = add_root(root)
        joints = mjcf_root.find_all("joint")
        self.assertEqual(6, len(joints))
        name_possibilities = ["test_joint", "test_joint_0", "test_joint_1"]
        for joint in joints:
            if hasattr(joint, "type") and joint.type == "revolute":
                self.assertIn(joint.name, name_possibilities)


if __name__ == "__main__":
    unittest.main()
