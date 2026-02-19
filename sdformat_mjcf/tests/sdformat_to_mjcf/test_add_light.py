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
from numpy.testing import assert_allclose
import math

from dm_control import mjcf

from gz.math import Angle, Color, Pose3d, Vector3d

import sdformat as sdf

from sdformat_mjcf.sdformat_to_mjcf.converters.light import add_light
from sdformat_mjcf.sdformat_to_mjcf.converters.link import add_link

from tests import helpers


class LightTest(helpers.TestCase):

    test_pose = Pose3d(1, 2, 3, math.pi / 2, math.pi / 3, math.pi / 4)
    expected_pos = [1., 2., 3.]
    expected_euler = [90., 60., 45.]

    def setUp(self):
        self.mujoco = mjcf.RootElement(model="test")
        self.body = self.mujoco.worldbody.add("body")

    def test_light(self):
        light = sdf.Light()
        light.set_name("light")
        light.set_type(sdf.LightType.DIRECTIONAL)
        light.set_raw_pose(self.test_pose)
        light.set_cast_shadows(True)
        light.set_diffuse(Color(0.4, 0.5, 0.6, 1.0))
        light.set_specular(Color(0.8, 0.9, 0.1, 1.0))
        light.set_linear_attenuation_factor(0.1)
        light.set_constant_attenuation_factor(0.5)
        light.set_quadratic_attenuation_factor(0.01)
        light.set_direction(Vector3d(0.1, 0.2, 1))
        light.set_intensity(1.7)
        light.set_light_on(True)

        light_mjcf = add_light(self.body, light)

        self.assertNotEqual(light_mjcf, None)
        self.assertEqual(light_mjcf.name, light.name())
        # Check type string
        self.assertEqual(light_mjcf.type, "directional")
        self.assertEqual(bool(light_mjcf.castshadow), light.cast_shadows())
        self.assertEqual(bool(light_mjcf.active), light.light_on())
        self.assertEqual(light_mjcf.intensity, light.intensity())
        
        assert_allclose(self.expected_pos, light_mjcf.pos)
        assert_allclose([light.constant_attenuation_factor(),
                         light.linear_attenuation_factor(),
                         light.quadratic_attenuation_factor()],
                        light_mjcf.attenuation)
        assert_allclose([0.864937, -0.549277, 0.013397],
                        light_mjcf.dir,
                        rtol=1e-4)
        assert_allclose([light.diffuse().r(),
                         light.diffuse().g(),
                         light.diffuse().b()],
                        light_mjcf.diffuse)
        assert_allclose([light.specular().r(),
                         light.specular().g(),
                         light.specular().b()],
                        light_mjcf.specular)

    def test_light_spot(self):
        light = sdf.Light()
        light.set_name("spotlight")
        light.set_type(sdf.LightType.SPOT)
        light.set_raw_pose(self.test_pose)
        light.set_spot_outer_angle(Angle(math.radians(30)))
        light.set_attenuation_range(20.0)
        light.set_intensity(0.8)
        light.set_spot_inner_angle(Angle(0.0))
        light.set_spot_falloff(1.0)

        light_mjcf = add_light(self.body, light)

        self.assertNotEqual(light_mjcf, None)
        self.assertEqual(light_mjcf.type, "spot")
        assert_allclose(light_mjcf.cutoff, 30.0)
        assert_allclose(light_mjcf.range, 20.0)
        assert_allclose(light_mjcf.intensity, 0.8)
        # Expected exponent calculated: Inner=0, Outer=30, Falloff=1.0 -> 19.99
        assert_allclose(light_mjcf.exponent, 20.0, rtol=1e-2)

    def test_light_point(self):
        light = sdf.Light()
        light.set_name("pointlight")
        light.set_type(sdf.LightType.POINT)
        light.set_raw_pose(self.test_pose)
        light.set_attenuation_range(5.0)

        light_mjcf = add_light(self.body, light)
        
        self.assertNotEqual(light_mjcf, None)
        self.assertEqual(light_mjcf.type, "spot")
        assert_allclose(light_mjcf.cutoff, 180.0)
        assert_allclose(light_mjcf.range, 5.0)

    def test_light_none(self):
        light_mjcf = add_light(self.body, None)
        self.assertEqual(None, light_mjcf)

    def test_link_light(self):
        link = sdf.Link()
        link.set_name("base_link")

        light = sdf.Light()
        light.set_name("light")
        light.set_type(sdf.LightType.DIRECTIONAL)
        light.set_raw_pose(self.test_pose)
        light.set_cast_shadows(True)
        light.set_diffuse(Color(0.4, 0.5, 0.6, 1.0))
        light.set_specular(Color(0.8, 0.9, 0.1, 1.0))
        light.set_linear_attenuation_factor(0.1)
        light.set_constant_attenuation_factor(0.5)
        light.set_quadratic_attenuation_factor(0.01)
        light.set_direction(Vector3d(0.1, 0.2, 1))
        light.set_intensity(1.7)

        link.add_light(light)

        body_mjcf = add_link(self.body, link)

        self.assertNotEqual(body_mjcf.light[0], None)
        self.assertEqual(body_mjcf.light[0].name, light.name())
        self.assertEqual(body_mjcf.light[0].type, "directional")
        self.assertEqual(bool(body_mjcf.light[0].castshadow),
                         light.cast_shadows())
        assert_allclose(self.expected_pos, body_mjcf.light[0].pos)
        assert_allclose([light.constant_attenuation_factor(),
                         light.linear_attenuation_factor(),
                         light.quadratic_attenuation_factor()],
                        body_mjcf.light[0].attenuation)
        assert_allclose([0.864937, -0.549277, 0.013397],
                        body_mjcf.light[0].dir,
                        rtol=1e-4)
        assert_allclose([light.diffuse().r(),
                         light.diffuse().g(),
                         light.diffuse().b()],
                        body_mjcf.light[0].diffuse)
        assert_allclose([light.specular().r(),
                         light.specular().g(),
                         light.specular().b()],
                        body_mjcf.light[0].specular)
        assert_allclose(body_mjcf.light[0].intensity, 1.7)


if __name__ == "__main__":
    unittest.main()
