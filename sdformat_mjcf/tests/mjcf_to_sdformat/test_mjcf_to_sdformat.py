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
import tempfile
import unittest
from math import pi

import sdformat as sdf
from ignition.math import Vector3d

from sdformat_mjcf.mjcf_to_sdformat.mjcf_to_sdformat import (
    mjcf_file_to_sdformat)
from sdformat_mjcf.mjcf_to_sdformat.converters.geometry import MESH_OUTPUT_DIR
from sdformat_mjcf.mjcf_to_sdformat.converters.material import (
    TEXTURE_OUTPUT_DIR)

from tests.helpers import get_resources_dir

TEST_RESOURCES_DIR = get_resources_dir()


class MjcfFileConversion(unittest.TestCase):

    def test_mesh_and_texture_output(self):
        input_file = str(TEST_RESOURCES_DIR / "mug.xml")
        with tempfile.TemporaryDirectory() as temp_dir:
            output_file = os.path.join(temp_dir, "mug.sdf")
            mjcf_file_to_sdformat(input_file, output_file)

            self.assertTrue(os.path.exists(output_file))
            root = sdf.Root()
            root.load(output_file)
            world = root.world_by_index(0)
            mug_model = world.model_by_name("model_for_mug")
            mug_link = mug_model.link_by_name("mug")

            def check_for_mesh(sdf_geom):
                self.assertEqual(sdf.GeometryType.MESH, sdf_geom.type())
                mesh_shape = sdf_geom.mesh_shape()
                self.assertIsNotNone(mesh_shape)
                expected_path = os.path.join(MESH_OUTPUT_DIR, "mug.obj")
                self.assertEqual(expected_path, mesh_shape.uri())
                self.assertTrue(
                    os.path.exists(os.path.join(temp_dir, expected_path)))

            mug_visual = mug_link.visual_by_index(0)
            self.assertEqual(Vector3d(pi / 2, 0, 0),
                             mug_visual.raw_pose().rot().euler())
            check_for_mesh(mug_visual.geometry())
            pbr = mug_visual.material().pbr_material()
            workflow = pbr.workflow(sdf.PbrWorkflowType.METAL)
            expected_texture_path = os.path.join(TEXTURE_OUTPUT_DIR, "mug.png")
            self.assertEqual(expected_texture_path, workflow.albedo_map())
            self.assertTrue(
                os.path.exists(os.path.join(temp_dir, expected_texture_path)))

            mug_collision = mug_link.collision_by_index(0)
            check_for_mesh(mug_collision.geometry())
            self.assertEqual(Vector3d(pi / 2, 0, 0),
                             mug_collision.raw_pose().rot().euler())


if __name__ == "__main__":
    unittest.main()
