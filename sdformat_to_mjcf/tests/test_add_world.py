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
from ignition.math import Color, Inertiald, Pose3d, MassMatrix3d, Vector3d
from dm_control import mjcf

from sdformat_to_mjcf.converters.world import add_world
from tests import helpers

GeometryType = sdf.Geometry.GeometryType


class WorldTest(helpers.TestCase):

    test_pose = Pose3d(1, 2, 3, pi / 2, pi / 3, pi / 4)
    expected_pos = [1.0, 2.0, 3.0]
    expected_euler = [90.0, 60.0, 45.0]

    def setUp(self):
        self.mujoco = mjcf.RootElement(model="test")
        self.body = self.mujoco.worldbody.add("body")

    def test_world(self):
        world = sdf.World()
        model = sdf.Model()
        model.set_raw_pose(self.test_pose)
        link = sdf.Link()
        link.set_name("base_link")
        link.set_raw_pose(self.test_pose)

        visual = sdf.Visual()
        visual.set_name("v1")
        visual.set_raw_pose(Pose3d(2, 3, 4, pi / 2, pi / 3, pi / 4))
        collision = sdf.Collision()
        collision.set_name("c1")
        collision.set_raw_pose(Pose3d(4, 2, 1, pi / 2, pi / 3, pi / 4))

        geometry = sdf.Geometry()
        geometry.set_box_shape(sdf.Box())
        geometry.set_type(GeometryType.BOX)
        visual.set_geometry(geometry)
        collision.set_geometry(geometry)

        link.add_visual(visual)
        link.add_collision(collision)

        light = sdf.Light()
        light.set_name("light")
        light.set_type(sdf.Light.LightType.DIRECTIONAL)
        light.set_raw_pose(self.test_pose)
        light.set_cast_shadows(True)
        light.set_diffuse(Color(0.4, 0.5, 0.6, 1.0))
        light.set_specular(Color(0.8, 0.9, 0.1, 1.0))
        light.set_linear_attenuation_factor(0.1)
        light.set_constant_attenuation_factor(0.5)
        light.set_quadratic_attenuation_factor(0.01)
        light.set_direction(Vector3d(0.1, 0.2, 1))
        light.set_intensity(1.7)

        world.add_light(light)

        # TODO(ahcorde): Add link light test

        model.add_link(link)
        world.add_model(model)

        self.body = add_world(self.mujoco, world)

        self.assertIsNotNone(self.mujoco.worldbody.body[1])
        assert_allclose([4.69962, 1.45698, 3.13397],
                        self.mujoco.worldbody.body[1].pos, rtol=1e-4)
        assert_allclose([-166.935687, 7.435472, 105.852574],
                        self.mujoco.worldbody.body[1].euler)
        geoms = self.body.find_all('geom')
        self.assertEqual(2, len(self.mujoco.worldbody.body[1].geom))
        self.assertEqual("base_link_v1",
                         self.mujoco.worldbody.body[1].geom[1].name)
        assert_allclose([2, 3, 4],
                        self.mujoco.worldbody.body[1].geom[1].pos)
        self.assertEqual("base_link_c1",
                         self.mujoco.worldbody.body[1].geom[0].name)
        assert_allclose([4, 2, 1],
                        self.mujoco.worldbody.body[1].geom[0].pos)

        self.assertNotEqual(self.mujoco.worldbody.light[0], None)
        self.assertEqual(self.mujoco.worldbody.light[0].name, light.name())
        self.assertEqual(bool(self.mujoco.worldbody.light[0].directional),
                         light.type() == sdf.Light.LightType.DIRECTIONAL)
        self.assertEqual(bool(self.mujoco.worldbody.light[0].castshadow),
                         light.cast_shadows())
        assert_allclose(self.expected_pos, self.mujoco.worldbody.light[0].pos)
        assert_allclose([light.constant_attenuation_factor(),
                         light.linear_attenuation_factor(),
                         light.quadratic_attenuation_factor()],
                        self.mujoco.worldbody.light[0].attenuation)
        assert_allclose([0.864937, -0.549277, 0.013397],
                        self.mujoco.worldbody.light[0].dir,
                        rtol=1e-4)
        assert_allclose([light.diffuse().r(),
                         light.diffuse().g(),
                         light.diffuse().b()],
                        self.mujoco.worldbody.light[0].diffuse)
        assert_allclose([light.specular().r(),
                         light.specular().g(),
                         light.specular().b()],
                        self.mujoco.worldbody.light[0].specular)

if __name__ == "__main__":
    unittest.main()
