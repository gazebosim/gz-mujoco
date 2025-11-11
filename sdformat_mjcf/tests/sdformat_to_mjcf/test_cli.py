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

import unittest
from contextlib import redirect_stderr
import io
import tempfile
import os
from os import path
import numpy as np
from dm_control import mujoco

from sdformat_mjcf.sdformat_to_mjcf import cli
from tests.helpers import get_resources_dir

TEST_RESOURCES_DIR = get_resources_dir()


class CLITest(unittest.TestCase):

    def test_missing_args(self):
        output = io.StringIO()
        with self.assertRaises(SystemExit), redirect_stderr(output):
            cli.main([])
        self.assertIn("arguments are required: input_file, output_file",
                      output.getvalue())

    def test_basic_conversion(self):
        output = io.StringIO()
        model_file = TEST_RESOURCES_DIR / "double_pendulum.sdf"
        with tempfile.TemporaryDirectory() as temp_dir:
            output_file = path.join(temp_dir, "double_pendulum.xml")
            with redirect_stderr(output):
                return_code = cli.main([str(model_file), output_file])
            self.assertEqual("", output.getvalue())
            self.assertEqual(0, return_code)

            # Check that the generated file is valid by loading it and
            # simulating a few steps
            engine = mujoco.Physics.from_xml_path(output_file)
            initial_state = engine.get_state()
            for i in range(100):
                engine.step()
            final_state = engine.get_state()
            self.assertFalse(np.allclose(initial_state, final_state,
                                         rtol=1e-6))

    def test_invalid_sdformat_file(self):
        output = io.StringIO()
        model_file = TEST_RESOURCES_DIR / "invalid_xml_syntax.sdf"
        with tempfile.TemporaryDirectory() as temp_dir:
            output_file = path.join(temp_dir, "test_invalid_file.xml")
            with redirect_stderr(output):
                return_code = cli.main([str(model_file), output_file])
            self.assertIn("Unable to read file", output.getvalue())
            self.assertEqual(1, return_code)

    def test_relative_output_dir(self):
        model_file = TEST_RESOURCES_DIR / "double_pendulum.sdf"
        with tempfile.TemporaryDirectory() as temp_dir:
            os.chdir(temp_dir)
            os.mkdir("mjcf")
            output_file = path.join(os.getcwd(), "mjcf", "double_pendulum.xml")
            return_code = cli.main([str(model_file), output_file])
            self.assertEqual(0, return_code)
            self.assertTrue(os.path.exists(output_file))

    def test_relative_resource_path(self):
        model_dir = TEST_RESOURCES_DIR / "box_obj"
        model_file = model_dir / "model.sdf"
        with tempfile.TemporaryDirectory() as temp_dir:
            os.chdir(temp_dir)
            os.mkdir("mjcf")
            self.assertNotEqual(os.getcwd(), str(model_dir))
            output_file = path.join(os.getcwd(), "mjcf", "box_obj.xml")
            return_code = cli.main([str(model_file), output_file])
            self.assertEqual(0, return_code)
            self.assertTrue(os.path.exists(output_file))


if __name__ == "__main__":
    unittest.main()
