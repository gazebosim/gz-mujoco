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

from dm_control import mjcf
from dm_control import mujoco

from sdformat_mjcf.mjcf_to_sdformat.converters.world import (
    mjcf_worldbody_to_sdf,
)

import sdformat as sdf


def mjcf_file_to_sdformat(input_file, output_file, export_world_plugins=True):
    """
    Loads a MJCF input file and converts to SDFormat.
    :param str input_file: Path to input MJCF file
    :param str output_file: Path to output SDFormat file.
    :param str export_world_plugins: If true SDFormat will export world plugins
    """
    mjcf_model = mjcf.from_path(input_file, escape_separators=True)
    physics = mujoco.Physics.from_xml_path(input_file)

    root = sdf.Root()
    world = sdf.World()
    world.set_name("default")

    mjcf_worldbody_to_sdf(mjcf_model, physics, world, export_world_plugins)

    root.add_world(world)

    with open(output_file, "w") as f:
        f.write(root.to_string())
