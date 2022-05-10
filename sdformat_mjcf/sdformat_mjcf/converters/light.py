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

import sdformat_mjcf.sdf_utils as su
import sdformat as sdf


def add_light(body, light, pose,
              pose_resolver=su.pose_resolver):
    """
    Converts an SDFormat light to an MJCF light and add it to the given body.

    :param mjcf.Element body: The MJCF body to which the geom is added.
    :param sdf.Light light: SDF light
    :param sdformat.Pose3d pose: Resolved pose of the light (obtained from the
            pose of the SDFormat Light).
    :return: The newly created MJCF light.
    :rtype: mjcf.Element
    """
    if light is None:
        return

    light = body.add("light",
                    name=light.name(),
                    pos=su.vec3d_to_list(pose.pos()),
                    directional=sdf.Light.LightType.DIRECTIONAL == light.type(),
                    castshadow=light.cast_shadows(),
                    attenuation=[light.constant_attenuation_factor(),
                                 light.linear_attenuation_factor(),
                                 light.quadratic_attenuation_factor()],
                    dir=su.vec3d_to_list(light.direction()),
                    diffuse=[light.diffuse().r(),
                             light.diffuse().g(),
                             light.diffuse().b()],
                    specular=[light.specular().r(),
                              light.specular().g(),
                              light.specular().b()])
    return light
