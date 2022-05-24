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
import os

import sdformat as sdf
from dm_control import mjcf

from sdformat_to_mjcf.converters.world import add_world
from tests import helpers
import sdformat_mjcf_utils.sdf_utils as su


class WorldTest(helpers.TestCase):

    def setUp(self):
        self.mujoco = mjcf.RootElement(model="test")
        self.body = self.mujoco.worldbody.add("body")

    def test_world(self):
        root = sdf.Root()
        root.load(os.path.join(
                  os.path.dirname(os.path.abspath(__file__)),
                  "resources/lights.sdf"))

        world = root.world_by_index(0)

        self.body = add_world(self.mujoco, world)

        assert_allclose(su.vec3d_to_list(world.gravity()),
                        self.mujoco.option.gravity)
        assert_allclose(su.vec3d_to_list(world.magnetic_field()),
                        self.mujoco.option.magnetic)
        assert_allclose(su.vec3d_to_list(world.wind_linear_velocity()),
                        self.mujoco.option.wind)
        self.assertEqual(3, len(self.mujoco.worldbody.body))

        assert_allclose([0, 0, 0.5],
                        self.mujoco.worldbody.body[1].pos, rtol=1e-4)
        assert_allclose([0, 0, 0],
                        self.mujoco.worldbody.body[1].euler)
        self.assertEqual(2, len(self.mujoco.worldbody.body[1].geom))
        self.assertEqual("box_box_link_box_visual",
                         self.mujoco.worldbody.body[1].geom[1].name)
        assert_allclose([0.5, 0, 0],
                        self.mujoco.worldbody.body[1].geom[1].pos)
        self.assertEqual("box_box_link_box_collision",
                         self.mujoco.worldbody.body[1].geom[0].name)
        assert_allclose([0, 0.5, 0.5],
                        self.mujoco.worldbody.body[1].geom[0].pos)

        assert_allclose([0, 1.5, 0.5],
                        self.mujoco.worldbody.body[2].pos, rtol=1e-4)
        assert_allclose([0, 0, 0],
                        self.mujoco.worldbody.body[2].euler)
        self.assertEqual(2, len(self.mujoco.worldbody.body[1].geom))
        self.assertEqual("sphere_sphere_link_sphere_visual",
                         self.mujoco.worldbody.body[2].geom[1].name)
        assert_allclose([0.0, 0, 0],
                        self.mujoco.worldbody.body[2].geom[1].pos)
        self.assertEqual("sphere_sphere_link_sphere_collision",
                         self.mujoco.worldbody.body[2].geom[0].name)
        assert_allclose([0, 0., 0.],
                        self.mujoco.worldbody.body[2].geom[0].pos)

        self.assertNotEqual(self.mujoco.worldbody.light[0], None)
        self.assertEqual("directional", self.mujoco.worldbody.light[0].name)
        self.assertEqual("true", self.mujoco.worldbody.light[0].directional)
        self.assertEqual("true", self.mujoco.worldbody.light[0].castshadow)
        assert_allclose([0., 0., 10.], self.mujoco.worldbody.light[0].pos)
        assert_allclose([0.9, 0.01, 0.001],
                        self.mujoco.worldbody.light[0].attenuation)
        assert_allclose([0.5, 0.2, -0.9],
                        self.mujoco.worldbody.light[0].dir,
                        rtol=1e-4)
        assert_allclose([0.8, 0.8, 0.8],
                        self.mujoco.worldbody.light[0].diffuse)
        assert_allclose([0.2, 0.2, 0.2],
                        self.mujoco.worldbody.light[0].specular)

    def test_world_link_mass_zero(self):
        root = sdf.Root()
        root.load(os.path.join(
                  os.path.dirname(os.path.abspath(__file__)),
                  "resources/model_mass_zero.sdf"))

        world = root.world_by_index(0)
        with self.assertRaises(RuntimeError):
            add_world(self.mujoco, world)


if __name__ == "__main__":
    unittest.main()
