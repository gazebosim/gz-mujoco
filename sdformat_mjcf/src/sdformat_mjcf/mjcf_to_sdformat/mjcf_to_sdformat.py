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

import os

from dm_control import mjcf
from dm_control import mujoco
from dm_control.mujoco.wrapper import util
import sdformat as sdf

from sdformat_mjcf.mjcf_to_sdformat.converters.world import (
    mjcf_worldbody_to_sdf,
)
from sdformat_mjcf.mjcf_to_sdformat.converters.geometry import MESH_OUTPUT_DIR
from sdformat_mjcf.mjcf_to_sdformat.converters.material import (
    TEXTURE_OUTPUT_DIR)

import sdformat_mjcf.utils.sdf_utils as su


def mjcf_file_to_sdformat(input_file, output_file, export_world_plugins=True):
    """
    Loads a MJCF input file and converts to SDFormat.
    :param str input_file: Path to input MJCF file
    :param str output_file: Path to output SDFormat file.
    :param str export_world_plugins: If true SDFormat will export world plugins
    """
    mjcf_model = mjcf.from_path(input_file)
    physics = mujoco.Physics.from_xml_path(input_file)

    root = sdf.Root()
    world = sdf.World()
    world.set_name("default")

    mjcf_worldbody_to_sdf(mjcf_model, physics, world, export_world_plugins)

    root.add_world(world)

    with open(output_file, "w") as f:
        f.write(root.to_string())

    # Export all assets to the directory that contains the output_file
    out_dir = os.path.dirname(output_file)

    asset_output = {
        MESH_OUTPUT_DIR: list(mjcf_model.asset.mesh),
        TEXTURE_OUTPUT_DIR: list(mjcf_model.asset.texture),
    }

    for sub_dir, assets in asset_output.items():
        new_dir = os.path.join(out_dir, sub_dir)
        for asset in assets:
            if asset.file is not None:
                filename = su.get_asset_filename_on_disk(asset)
                if not os.path.exists(new_dir):
                    os.makedirs(new_dir)
                with open(os.path.join(new_dir, filename), 'wb') as f:
                    f.write(util.to_binary_string(asset.file.contents))
