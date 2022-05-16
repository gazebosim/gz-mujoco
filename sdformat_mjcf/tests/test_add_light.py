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

from ignition.math import Color, Pose3d, Vector3d

from dm_control import mjcf

import sdformat as sdf

from sdformat_mjcf.converters.light import add_light


class LightTest(unittest.TestCase):

    test_pose = Pose3d(1, 2, 3, math.pi / 2, math.pi / 3, math.pi / 4)
    expected_pos = [1., 2., 3.]
    expected_euler = [90., 60., 45.]

    def test_light(self):
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

        mujoco = mjcf.RootElement(model="test")
        body = mujoco.worldbody.add('body')

        light_mjcf = add_light(body, light, self.test_pose)

        self.assertNotEqual(light_mjcf, None)
        self.assertEqual(light_mjcf.name, light.name())
        self.assertEqual(bool(light_mjcf.directional),
                         light.type() == sdf.Light.LightType.DIRECTIONAL)
        self.assertEqual(bool(light_mjcf.castshadow), light.cast_shadows())
        assert_allclose(self.expected_pos, light_mjcf.pos)
        assert_allclose([light.constant_attenuation_factor(),
                         light.linear_attenuation_factor(),
                         light.quadratic_attenuation_factor()],
                        light_mjcf.attenuation)
        assert_allclose([0.752613, 0.190018, 0.43921],
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

    def test_light_none(self):
        mujoco = mjcf.RootElement(model="test")
        body = mujoco.worldbody.add('body')

        light_mjcf = add_light(body, None, self.test_pose)
        self.assertEqual(None, light_mjcf)


if __name__ == "__main__":
    unittest.main()
