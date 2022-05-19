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

import argparse
import os
import sys

import sdformat as sdf
from dm_control import mjcf
from dm_control.mjcf.export_with_assets import export_with_assets
from sdformat_mjcf.converters.model import add_model
from sdformat_mjcf.converters.world import add_world


def sdformat_root_to_mjcf(root):
    mjcf_root = mjcf.RootElement()
    mjcf_root.compiler.eulerseq = 'XYZ'
    mjcf_root.model = root.model().name()
    if root.model():
        mjcf_root.model = root.model().name()
    elif root.world_count() == 1:
        mjcf_root.model = "default"
    else:
        raise RuntimeError("One model or one world is supported")

    if root.model():
        asset = mjcf_root.asset
        asset.add("texture",
                  name="grid",
                  type="2d",
                  builtin="checker",
                  width=512,
                  height=512,
                  rgb1=[.1, .2, .3],
                  rgb2=[.2, .3, .4])
        asset.add("material",
                  name="grid",
                  texture="grid",
                  texrepeat="1 1",
                  texuniform=True,
                  reflectance=0.2)

        mjcf_root.worldbody.add("geom",
                                name="floor",
                                size=[0, 0, .05],
                                pos=[0, 0, -1],
                                type="plane",
                                material="grid",
                                condim=3)

        return add_model(mjcf_root, root.model())
    elif root.world_count() > 0:
        # Does not support multiple worlds
        return add_world(mjcf_root, root.world_by_index(0))


def sdformat_file_to_mjcf(model_file, output_file):
    root = sdf.Root()
    errors = root.load(model_file)
    if errors:
        print(errors, file=sys.stderr)
    else:
        mjcf_root = sdformat_root_to_mjcf(root)
        export_with_assets(mjcf_root, os.path.dirname(output_file),
                           output_file)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("model_file")
    parser.add_argument("output_file")

    args = parser.parse_args()
    sdformat_file_to_mjcf(args.model_file, args.output_file)
