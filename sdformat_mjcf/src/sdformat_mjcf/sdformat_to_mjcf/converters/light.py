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

"""Module to convert SDFormat Lights to MJCF"""


import math
import sdformat_mjcf.utils.sdf_utils as su
import sdformat as sdf


def add_light(body, light):
    """
    Converts an SDFormat light to an MJCF light and add it to the given body.

    :param mjcf.Element body: The MJCF body to which the geom is added.
    :param sdf.Light light: SDF light
    :return: The newly created MJCF light.
    :rtype: mjcf.Element
    """
    if light is None:
        return

    # Convert SDF LightType enum to MJCF type string
    type_str = str(light.type()).split(".")[-1].lower()

    # MuJoCo only supports spot and directional lights.
    # Map point lights to spot lights with 180 degree cutoff.
    if type_str == "point":
        type_str = "spot"

    pose = su.graph_resolver.resolve_pose(light.semantic_pose())

    # Prepare attributes
    attributes = {
        "name": su.find_unique_name(body, "light", light.name()),
        "pos": su.vec3d_to_list(pose.pos()),
        "type": type_str,
        "castshadow": light.cast_shadows(),
        "active": light.light_on(),
        "attenuation": [light.constant_attenuation_factor(),
                        light.linear_attenuation_factor(),
                        light.quadratic_attenuation_factor()],
        "dir": su.vec3d_to_list(pose.rot() * light.direction()),
        "diffuse": [light.diffuse().r(),
                    light.diffuse().g(),
                    light.diffuse().b()],
        "specular": [light.specular().r(),
                     light.specular().g(),
                     light.specular().b()],
        "intensity": light.intensity()
    }

    # Handle spot light specific attributes and point light mapping
    if type_str == "spot":
        attributes["range"] = light.attenuation_range()
        if light.type() == sdf.LightType.POINT:
            # Point lights are spot lights with 360 degree cutoff
            attributes["cutoff"] = 360.0
        else:
            attributes["cutoff"] = light.spot_outer_angle().degree()

            # Calculate exponent to match the 50% intensity point.
            # SDFormat attenuates intensity between inner and outer angles.
            # The base attenuation is linear with angle, raised to the falloff power:
            # I = ((outer - angle) / (outer - inner)) ^ falloff
            # MuJoCo uses the OpenGL fixed-function spotlight model:
            # I = cos(angle) ^ exponent
            # We find the angle 'x' where SDF intensity is 0.5, and then solve for
            # the MuJoCo exponent that gives 0.5 intensity at that same angle.
            outer_rad = light.spot_outer_angle().radian()
            inner_rad = light.spot_inner_angle().radian()
            falloff = light.spot_falloff()

            exponent = 0.0
            if falloff > 1e-6 and outer_rad > 1e-6:
                # 1. Find angle 'x' where SDF intensity is 0.5
                # 0.5 = ((outer - x) / (outer - inner)) ^ falloff
                # 0.5 ^ (1/falloff) = (outer - x) / (outer - inner)
                ratio = math.pow(0.5, 1.0 / falloff)
                x = outer_rad - (outer_rad - inner_rad) * ratio

                # 2. Solve for exponent where cos(x)^exponent = 0.5
                # exponent = ln(0.5) / ln(cos(x))
                cos_x = math.cos(x)
                if cos_x > 1e-6:
                    denom = math.log(cos_x)
                    if abs(denom) > 1e-6:
                        exponent = math.log(0.5) / denom

            attributes["exponent"] = exponent

    light = body.add("light", **attributes)
    return light
