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

from ignition.math import Color, Pose3d, Vector3d, Quaterniond

import sdformat_mjcf.sdf_utils as su

import sdformat as sdf

LIGHT_NUMBER = 0


def add_mjcf_light_to_sdf(light):
    light_sdf = sdf.Light()
    if light.name is not None:
        light_sdf.set_name(light.name)
    else:
        global LIGHT_NUMBER
        light_sdf.set_name("light_" + str(LIGHT_NUMBER))
        LIGHT_NUMBER = LIGHT_NUMBER + 1

    if light.pos is not None:
        light_sdf.set_raw_pose(Pose3d(su.list_to_vec3d(light.pos),
                                      Quaterniond()))
    if light.castshadow is not None:
        light_sdf.set_cast_shadow(bool(light.castshadow))

    if light.attenuation is not None:
        light_sdf.set_constant_attenuation_factor(light.attenuation[0])
        light_sdf.set_linear_attenuation_factor(light.attenuation[1])
        light_sdf.set_quadratic_attenuation_factor(light.attenuation[2])

    if light.diffuse is not None:
        light_sdf.set_diffuse(Color(light.diffuse[0],
                                    light.diffuse[1],
                                    light.diffuse[2]))
    if light.specular is not None:
        light_sdf.set_specular(Color(light.specular[0],
                                     light.specular[1],
                                     light.specular[2]))
    if light.directional is not None:
        if light.directional == "true":
            light_sdf.set_type(sdf.Light.LightType.DIRECTIONAL)
        else:
            light_sdf.set_type(sdf.Light.LightType.SPOT)

    if light.dir is not None:
        light_sdf.set_direction(su.list_to_vec3d(light.dir))

    return light_sdf
