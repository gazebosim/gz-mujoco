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

import tempfile
import unittest
import os
from math import pi

import sdformat as sdf
from gz.math import Pose3d
from dm_control import mjcf

from sdformat_mjcf.sdformat_to_mjcf.converters.root import add_root
from sdformat_mjcf.sdformat_to_mjcf.sdformat_to_mjcf import (
    sdformat_file_to_mjcf)
from tests import helpers

TEST_RESOURCES_DIR = helpers.get_resources_dir()


class RootTest(helpers.TestCase):
    test_pose = Pose3d(1, 2, 3, pi / 2, pi / 3, pi / 4)
    expected_pos = [1.0, 2.0, 3.0]
    expected_euler = [90.0, 60.0, 45.0]

    def setUp(self):
        self.mujoco = mjcf.RootElement(model="test")

    def test_root_with_model(self):
        root = sdf.Root()
        model = sdf.Model()
        model.set_name("test_model")
        model.set_static(True)
        root.set_model(model)

        mjcf_root = add_root(root)
        self.assertIsNotNone(mjcf_root)
        self.assertEqual("test_model", mjcf_root.model)

    def test_root_with_one_world(self):
        root = sdf.Root()
        world = sdf.World()
        world.set_name("test_world")
        root.add_world(world)

        mjcf_root = add_root(root)
        self.assertIsNotNone(mjcf_root)
        self.assertEqual("test_world", mjcf_root.model)

    def test_root_with_multiple_worlds(self):
        root = sdf.Root()
        world1 = sdf.World()
        world1.set_name("test_world1")
        root.add_world(world1)
        world2 = sdf.World()
        world2.set_name("test_world2")
        root.add_world(world2)

        with self.assertRaises(RuntimeError,
                               msg="One model or one world is supported"):
            add_root(root)

    def test_parsing_mjcf_output(self):
        model_file = str(TEST_RESOURCES_DIR / "double_pendulum.sdf")
        with tempfile.TemporaryDirectory() as temp_dir:
            output_file = os.path.join(temp_dir, "out.xml")
            sdformat_file_to_mjcf(model_file, output_file)
            print(open(output_file).read())
            model = mjcf.from_path(output_file)
            self.assertIsNotNone(model)


if __name__ == "__main__":
    unittest.main()
