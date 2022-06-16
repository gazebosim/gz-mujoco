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

from ignition.math import Angle, Color, Pose3d, Quaterniond, Vector3d

import math

import sdformat_mjcf.utils.sdf_utils as su

import sdformat as sdf

LIGHT_NUMBER = 0


def mjcf_light_to_sdf(light):
    """
    Converts a MJCF light in an SDFormat light.

    Default values are defined in the following reference:
     - https://mujoco.readthedocs.io/en/latest/XMLreference.html

    :param mjcf.Light light: The MJCF light
    :return: The newly created SDFormat light.
    :rtype: sdf.Light
    """
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
        light_sdf.set_cast_shadows(light.castshadow == "true")

    if light.attenuation is not None:
        light_sdf.set_constant_attenuation_factor(light.attenuation[0])
        light_sdf.set_linear_attenuation_factor(light.attenuation[1])
        light_sdf.set_quadratic_attenuation_factor(light.attenuation[2])
    else:
        light_sdf.set_constant_attenuation_factor(1.0)
        light_sdf.set_linear_attenuation_factor(0.0)
        light_sdf.set_quadratic_attenuation_factor(0.0)

    if light.diffuse is not None:
        light_sdf.set_diffuse(Color(light.diffuse[0],
                                    light.diffuse[1],
                                    light.diffuse[2]))
    else:
        light_sdf.set_diffuse(Color(0.7, 0.7, 0.7))

    if light.specular is not None:
        light_sdf.set_specular(Color(light.specular[0],
                                     light.specular[1],
                                     light.specular[2]))
    else:
        light_sdf.set_specular(Color(0.3, 0.3, 0.3))

    if light.active is not None:
        light_sdf.set_light_on(light.active == "true")

    def set_spot_light(light, light_sdf):
        light_sdf.set_type(sdf.LightType.SPOT)
        # The settings for inner_angle, outer_angle, and falloff were
        # determined experimentally.
        light_sdf.set_spot_inner_angle(Angle(0))

        # always in degrees regardless of the global angle setting.
        cutoff = su.get_value_or_default(light.cutoff, 45)
        light_sdf.set_spot_outer_angle(Angle(math.radians(cutoff * 2)))

        exponent = su.get_value_or_default(light.exponent, 10)
        light_sdf.set_spot_falloff(exponent * 2 / 10.0)

    if light.directional is not None:
        if light.directional == "true":
            light_sdf.set_type(sdf.LightType.DIRECTIONAL)
        else:
            set_spot_light(light, light_sdf)
    else:
        set_spot_light(light, light_sdf)

    if light.dir is not None:
        light_sdf.set_direction(su.list_to_vec3d(light.dir))
    else:
        light_sdf.set_direction(Vector3d(0, 0, -1))

    return light_sdf
