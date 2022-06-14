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
from sdformat_mjcf.sdformat_to_mjcf.sdformat_to_mjcf import (
    sdformat_file_to_mjcf,
)


def main(argv=None):
    """
    Main function that processes command line inputs.
    :param list[str] argv: (Optional) Command line arguments mainly used in
    testing. If argv is None, ArgumentParser will use sys.argv.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("input_file",
                        help="Path to input SDFormat file (World or Model)")
    parser.add_argument("output_file",
                        help="Desired path for the output MJCF file")

    args = parser.parse_args(argv)
    return sdformat_file_to_mjcf(args.input_file, args.output_file)


if __name__ == "__main__":
    sys.exit(main())
