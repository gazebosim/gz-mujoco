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

import argparse

from dm_control import mjcf

from sdformat_to_mjcf.converters.model import add_mjcf_model_to_sdf

import sdformat as sdf


def mjcf_file_to_sdformat(model_file):
    mjcf_model = mjcf.from_path(model_file)
    root = sdf.Root()
    world = sdf.World()
    world.set_name(mjcf_model.model)

    add_mjcf_model_to_sdf(mjcf_model, world)

    root.add_world(world)

    print(root.to_string())

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("model_file")

    args = parser.parse_args()
    mjcf_file_to_sdformat(args.model_file)
