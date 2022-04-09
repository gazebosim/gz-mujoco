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
from ignition.math import Inertiald, Pose3d, MassMatrix3d, Vector3d
from dm_control import mjcf

from sdformat_mjcf.converters.link import add_link
import helpers


class LinkTest(unittest.TestCase):
    test_pose = Pose3d(1, 2, 3, pi / 2, pi / 3, pi / 4)
    expected_pos = [1.0, 2.0, 3.0]
    expected_euler = [90.0, 60.0, 45.0]

    def setUp(self):
        self.mujoco = mjcf.RootElement(model="test")
        self.body = self.mujoco.worldbody.add("body")

    def moi_to_list(self, moi):
        return [
            moi(0, 0),
            moi(1, 1),
            moi(2, 2),
            moi(0, 1),
            moi(0, 2),
            moi(1, 2)
        ]

    def test_basic_link(self):
        link = sdf.Link()
        link.set_name("base_link")
        link.set_raw_pose(self.test_pose)
        mj_body = add_link(self.body,
                           link,
                           pose_resolver=helpers.nonthrowing_pose_resolver)
        self.assertIsNotNone(mj_body)
        assert_allclose(self.expected_pos, mj_body.pos)
        assert_allclose(self.expected_euler, mj_body.euler)

    def test_link_inertia(self):
        # This following is equivalent to
        # <link name="base_link">
        #   <inertial>
        #       <pose degrees="true">1 2 3   90 60 45</pose>
        #       <mass>10</mass>
        #       <inertia>
        #           <ixx>2.0</ixx>
        #           <iyy>3.0</iyy>
        #           <izz>4.0</izz>
        #       </inertia>
        #   </inertial>
        link = sdf.Link()
        link.set_name("base_link")
        inertial = Inertiald(
            MassMatrix3d(10, Vector3d(2.0, 3.0, 4.0), Vector3d.ZERO),
            self.test_pose)
        link.set_inertial(inertial)
        mj_body = add_link(self.body,
                           link,
                           pose_resolver=helpers.nonthrowing_pose_resolver)
        self.assertIsNotNone(mj_body)
        assert_allclose(self.moi_to_list(inertial.moi()),
                        mj_body.inertial.fullinertia)
        self.assertIsNone(mj_body.inertial.euler)


if __name__ == "__main__":
    unittest.main()