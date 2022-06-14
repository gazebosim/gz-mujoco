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

import sys
import argparse
from sdformat_mjcf.mjcf_to_sdformat.mjcf_to_sdformat import (
    mjcf_file_to_sdformat,
)


def main(argv=None):
    """
    Main function that processes command line inputs.
    :param list[str] argv: (Optional) Command line arguments mainly used in
    testing. If argv is None, ArgumentParser will use sys.argv.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("input_file",
                        help="Path to input MJCF file")
    parser.add_argument("output_file",
                        help="Desired path for the output SDFormat file")
    parser.add_argument('--export_world_plugins',
                        default=False,
                        action='store_true',
                        help="Export world plugins")
    args = parser.parse_args(argv)
    return mjcf_file_to_sdformat(args.input_file,
                                 args.output_file,
                                 args.export_world_plugins)


if __name__ == "__main__":
    sys.exit(main())
