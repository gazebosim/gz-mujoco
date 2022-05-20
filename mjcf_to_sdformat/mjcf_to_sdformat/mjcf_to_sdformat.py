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

from mjcf_to_sdformat.converters.model import add_mjcf_worldbody_to_sdf

import sdformat as sdf


def mjcf_file_to_sdformat(input_file, output_file):
    """
    Loads a MJCF input file and converts to SDFormat.
    :param str input_file: Path to input MJCF file
    :param str output_file: Path to output SDFormat file.
    """
    mjcf_model = mjcf.from_path(input_file)
    root = sdf.Root()
    world = sdf.World()
    world.set_name("default")

    add_mjcf_worldbody_to_sdf(mjcf_model, world)

    root.add_world(world)

    with open(output_file, "w") as f:
        f.write(root.to_string())
        f.close()
