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
from ignition.math import Pose3d, Vector3d

from tests import helpers
import sdformat_mjcf.sdf_utils as su


class GraphResolverTest(unittest.TestCase):

    test_pose = Pose3d(1, 2, 3, 0, 0, 0)
    test_xyz = Vector3d(0, 1, 0)

    def test_default(self):
        link = sdf.Link()
        link.set_name("base_link")
        link.set_raw_pose(self.test_pose)
        with self.assertRaises(RuntimeError):
            su.graph_resolver.resolve_pose(link.semantic_pose())

        joint_axis = sdf.JointAxis()
        joint_axis.set_xyz(self.test_xyz)
        with self.assertRaises(RuntimeError):
            su.graph_resolver.resolve_axis_xyz(joint_axis)

    def test_override(self):
        link = sdf.Link()
        link.set_name("base_link")
        link.set_raw_pose(self.test_pose)

        with helpers.test_graph_resolver_context():
            pose = su.graph_resolver.resolve_pose(link.semantic_pose())
            self.assertEqual(self.test_pose, pose)

        joint_axis = sdf.JointAxis()
        joint_axis.set_xyz(self.test_xyz)
        with helpers.test_graph_resolver_context():
            xyz = su.graph_resolver.resolve_axis_xyz(joint_axis)
            self.assertEqual(self.test_xyz, xyz)


if __name__ == "__main__":
    unittest.main()
