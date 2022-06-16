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

from dm_control import mjcf

from ignition.math import Color, Vector3d

import sdformat as sdf

from sdformat_mjcf.mjcf_to_sdformat.converters import light as light_conv


class LightTest(unittest.TestCase):

    def test_light_default(self):
        mujoco = mjcf.RootElement(model="test")
        body = mujoco.worldbody.add('body')
        light = body.add('light')

        sdf_light = light_conv.mjcf_light_to_sdf(light)
        self.assertEqual("light_0", sdf_light.name())
        self.assertEqual(Vector3d(0, 0, 0), sdf_light.raw_pose().pos())
        self.assertEqual(1, sdf_light.constant_attenuation_factor())
        self.assertEqual(0, sdf_light.linear_attenuation_factor())
        self.assertEqual(0, sdf_light.quadratic_attenuation_factor())
        self.assertEqual(Color(0.7, 0.7, 0.7), sdf_light.diffuse())
        self.assertEqual(Color(0.3, 0.3, 0.3), sdf_light.specular())
        self.assertEqual(sdf.LightType.SPOT, sdf_light.type())
        self.assertEqual(Vector3d(0, 0, -1), sdf_light.direction())
        self.assertAlmostEqual(0, sdf_light.spot_inner_angle().degree())
        self.assertAlmostEqual(90, sdf_light.spot_outer_angle().degree())
        self.assertEqual(2, sdf_light.spot_falloff())
        self.assertTrue(sdf_light.light_on())

    def test_light(self):
        mujoco = mjcf.RootElement(model="test")
        body = mujoco.worldbody.add('body')
        light = body.add('light',
                         name="light_mujoco",
                         pos=[0, 1, 2],
                         castshadow=False,
                         attenuation=[0.1, 0.3, 1.3],
                         diffuse=[0.8, 0.7, 0.6],
                         specular=[0.2, 0.2, 0.2],
                         directional="true",
                         dir=[0, -1, -1],
                         active="false")

        sdf_light = light_conv.mjcf_light_to_sdf(light)
        self.assertEqual("light_mujoco", sdf_light.name())
        self.assertEqual(Vector3d(0, 1, 2), sdf_light.raw_pose().pos())
        self.assertEqual(0.1, sdf_light.constant_attenuation_factor())
        self.assertEqual(0.3, sdf_light.linear_attenuation_factor())
        self.assertEqual(1.3, sdf_light.quadratic_attenuation_factor())
        self.assertEqual(Color(0.8, 0.7, 0.6), sdf_light.diffuse())
        self.assertEqual(Color(0.2, 0.2, 0.2), sdf_light.specular())
        self.assertEqual(sdf.LightType.DIRECTIONAL, sdf_light.type())
        self.assertEqual(Vector3d(0, -1, -1), sdf_light.direction())
        self.assertFalse(sdf_light.light_on())

    def test_light_none(self):
        mujoco = mjcf.RootElement(model="test")
        body = mujoco.worldbody.add('body')
        light = body.add('light',
                         name=None,
                         pos=[0, 1, 2],
                         castshadow=False,
                         attenuation=[0.1, 0.3, 1.3],
                         diffuse=[0.8, 0.7, 0.6],
                         specular=[0.2, 0.2, 0.2],
                         directional="false",
                         dir=[0, -1, -1],
                         active="true",
                         cutoff="40",
                         exponent="5")
        sdf_light = light_conv.mjcf_light_to_sdf(light)

        self.assertEqual("light_1", sdf_light.name())
        self.assertEqual(Vector3d(0, 1, 2), sdf_light.raw_pose().pos())
        self.assertEqual(0.1, sdf_light.constant_attenuation_factor())
        self.assertEqual(0.3, sdf_light.linear_attenuation_factor())
        self.assertEqual(1.3, sdf_light.quadratic_attenuation_factor())
        self.assertEqual(Color(0.8, 0.7, 0.6), sdf_light.diffuse())
        self.assertEqual(Color(0.2, 0.2, 0.2), sdf_light.specular())
        self.assertEqual(sdf.LightType.SPOT, sdf_light.type())
        self.assertEqual(Vector3d(0, -1, -1), sdf_light.direction())
        self.assertAlmostEqual(0, sdf_light.spot_inner_angle().degree())
        self.assertAlmostEqual(80, sdf_light.spot_outer_angle().degree())
        self.assertEqual(1, sdf_light.spot_falloff())
        self.assertTrue(sdf_light.light_on())


if __name__ == "__main__":
    unittest.main()
