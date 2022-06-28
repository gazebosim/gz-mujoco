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
import sys

import sdformat as sdf
from dm_control.mjcf.export_with_assets import export_with_assets
from sdformat_mjcf.sdformat_to_mjcf.converters.root import add_root


def sdformat_file_to_mjcf(input_file, output_file):
    """
    Loads an SDFormat input file and converts to MJCF.
    :param str input_file: Path to input SDFormat Model or World file
    :param str output_file: Path to output MJCF file. Any generated artifacts,
    such as meshes will be output to the directory containing the output file.
    """
    root = sdf.Root()
    try:
        root.load(input_file)
    except sdf.SDFErrorsException as e:
        print(e, file=sys.stderr)
        return 1
    else:
        mjcf_root = add_root(root)
        mjcf_root.default.dclass = "unused"
        output_dir, file_name = os.path.split(os.path.abspath(output_file))
        export_with_assets(mjcf_root, output_dir, file_name)
        return 0
