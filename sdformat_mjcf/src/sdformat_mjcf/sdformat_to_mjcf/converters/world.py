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

from sdformat_mjcf.sdformat_to_mjcf.converters.light import add_light
from sdformat_mjcf.sdformat_to_mjcf.converters.model import add_model
import sdformat_mjcf.utils.sdf_utils as su


def add_world(mjcf_out, world):
    """
    Convert a SDF world in a MJCF worldbody
    :param mjcf.Element mjcf_out: The MJCF body to which the models or lights
    are added.
    :param sdf.World world: The SDF World to convert
    """
    for mo in range(world.model_count()):
        model = world.model_by_index(mo)
        add_model(mjcf_out, model)
    for li in range(world.light_count()):
        light = world.light_by_index(li)
        add_light(mjcf_out.worldbody, light)

    # TODO(ahcorde) Add this properties
    # - audio_device
    # - spherical_coordinates ?
    # - atmosphere
    # - physics
    # - Scene ?

    mjcf_out.option.gravity = su.vec3d_to_list(world.gravity())
    mjcf_out.option.magnetic = su.vec3d_to_list(world.magnetic_field())
    mjcf_out.option.wind = su.vec3d_to_list(world.wind_linear_velocity())
    return mjcf_out
