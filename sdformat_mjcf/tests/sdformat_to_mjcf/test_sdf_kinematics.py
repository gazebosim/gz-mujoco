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

import sdformat13 as sdf
from sdformat_mjcf.sdformat_to_mjcf.sdf_kinematics import KinematicHierarchy
from tests.helpers import get_resources_dir

TEST_RESOURCES_DIR = get_resources_dir()


class TreeTest(unittest.TestCase):

    def load_sdf_file(self, file_name):
        root = sdf.Root()
        root.load(file_name)
        return root

    def test_hierarchy(self):
        model_file = TEST_RESOURCES_DIR / "double_pendulum.sdf"
        root = self.load_sdf_file(str(model_file))
        self.assertIsNotNone(root.model())
        kin_hierarchy = KinematicHierarchy(root.model())
        self.assertEqual(1, len(kin_hierarchy.world_node.child_nodes))

        base_node = kin_hierarchy.world_node.child_nodes[0]
        self.assertEqual("base", base_node.link.name())
        self.assertEqual(1, len(base_node.child_nodes))
        self.assertEqual("fix_to_world", base_node.joint.name())
        self.assertEqual(sdf.JointType.FIXED, base_node.joint.type())

        upper_link_node = base_node.child_nodes[0]
        self.assertEqual("upper_link", upper_link_node.link.name())
        self.assertEqual(1, len(upper_link_node.child_nodes))
        self.assertEqual("upper_joint", upper_link_node.joint.name())
        self.assertEqual(sdf.JointType.REVOLUTE, upper_link_node.joint.type())

        lower_link_node = upper_link_node.child_nodes[0]
        self.assertEqual("lower_link", lower_link_node.link.name())
        self.assertEqual(0, len(lower_link_node.child_nodes))
        self.assertEqual("lower_joint", lower_link_node.joint.name())
        self.assertEqual(sdf.JointType.REVOLUTE, lower_link_node.joint.type())


if __name__ == "__main__":
    unittest.main()
