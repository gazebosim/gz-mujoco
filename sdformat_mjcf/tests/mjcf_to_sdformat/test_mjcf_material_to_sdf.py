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

import os
import unittest

from gz.math7 import Color

from dm_control import mjcf
from dm_control import mujoco

from sdformat_mjcf.mjcf_to_sdformat.converters.material import (
    mjcf_material_to_sdf,
    TEXTURE_OUTPUT_DIR
)
from sdformat_mjcf.mjcf_to_sdformat.converters.world import (
    mjcf_worldbody_to_sdf,
)

import sdformat13 as sdf

from tests.helpers import get_resources_dir

TEST_RESOURCES_DIR = get_resources_dir()


class MaterialTest(unittest.TestCase):

    def test_material(self):
        mjcf_model = mjcf.from_path(
            str(TEST_RESOURCES_DIR / "test_mujoco.xml"))

        material = mjcf_material_to_sdf(mjcf_model.worldbody.geom[0])
        self.assertNotEqual(None, material)

        material = mjcf_material_to_sdf(mjcf_model.worldbody.body[0].geom[0])
        self.assertNotEqual(None, material)
        self.assertEqual(Color(0.36, 0.36, 0.36, 1.0), material.diffuse())
        self.assertEqual(Color(0.36, 0.36, 0.36, 1.0), material.ambient())
        self.assertEqual(Color(0.3, 0.3, 0.3, 1.0), material.specular())
        self.assertEqual(Color(0, 0, 0, 1.0), material.emissive())

        material = mjcf_material_to_sdf(mjcf_model.worldbody.body[1].geom[0])
        self.assertNotEqual(None, material)
        self.assertEqual(Color(0, 0.9, 0, 1.0), material.diffuse())
        self.assertEqual(Color(0, 0.9, 0, 1.0), material.ambient())
        self.assertEqual(Color(0, 0.9, 0, 1.0), material.specular())
        self.assertEqual(Color(0, 0.0, 0, 1.0), material.emissive())
        geom_default_mat = mjcf_model.find("geom", "default_mat")
        self.assertIsNotNone(geom_default_mat)
        material = mjcf_material_to_sdf(geom_default_mat)
        self.assertNotEqual(None, material)
        self.assertEqual(Color(1, 1, 1, 1.0), material.diffuse())
        self.assertEqual(Color(1, 1, 1, 1.0), material.ambient())
        self.assertEqual(Color(0.5, 0.5, 0.5, 1.0), material.specular())
        self.assertEqual(Color(0, 0, 0, 1.0), material.emissive())

    def test_material_texture(self):
        mjcf_model = mjcf.from_path(
            str(TEST_RESOURCES_DIR / "tennis_ball.xml"))

        material = mjcf_material_to_sdf(mjcf_model.worldbody.body[0].geom[0])
        self.assertNotEqual(None, material)
        pbr = material.pbr_material()
        self.assertNotEqual(None, pbr)
        workflow = pbr.workflow(sdf.PbrWorkflowType.METAL)
        self.assertNotEqual(None, workflow)
        self.assertNotEqual(None, workflow.albedo_map())

    def test_material_mjcf_worldbody_to_sdf(self):
        filename = str(TEST_RESOURCES_DIR / "tennis_ball.xml")
        mjcf_model = mjcf.from_path(filename)
        physics = mujoco.Physics.from_xml_path(filename)

        world = sdf.World()
        world.set_name("default")

        mjcf_worldbody_to_sdf(mjcf_model, physics, world)

        model = world.model_by_index(1)
        link = model.link_by_index(0)
        visual = link.visual_by_index(0)

        material = visual.material()
        self.assertNotEqual(None, material)
        self.assertEqual(Color(1, 1, 1, 1.0), material.diffuse())
        self.assertEqual(Color(1, 1, 1, 1.0), material.ambient())
        self.assertEqual(Color(0.5, 0.5, 0.5, 1.0), material.specular())
        self.assertEqual(Color(0, 0, 0, 1.0), material.emissive())
        pbr = material.pbr_material()
        self.assertNotEqual(None, pbr)
        workflow = pbr.workflow(sdf.PbrWorkflowType.METAL)
        self.assertNotEqual(None, workflow)
        expected_path = os.path.join(TEXTURE_OUTPUT_DIR, "tennis_ball.png")
        self.assertEqual(expected_path, workflow.albedo_map())

    def test_no_material(self):
        mjcf_string = """
        <mujoco>
            <worldbody>
                <geom type="box" size="1 1 1"/>
            </worldbody>
        </mujoco>
        """
        mjcf_model = mjcf.from_xml_string(mjcf_string)
        physics = mujoco.Physics.from_xml_string(mjcf_string)

        world = sdf.World()
        world.set_name("default")

        mjcf_worldbody_to_sdf(mjcf_model, physics, world)
        static_model = world.model_by_name("static")
        box_mat = static_model.link_by_index(0).visual_by_index(0).material()
        self.assertEqual(Color(0.5, 0.5, 0.5, 1.0), box_mat.diffuse())
        self.assertEqual(Color(0.5, 0.5, 0.5, 1.0), box_mat.ambient())
        self.assertEqual(Color(0.5, 0.5, 0.5, 1.0), box_mat.specular())
        self.assertEqual(Color(0, 0, 0, 1.0), box_mat.emissive())


if __name__ == "__main__":
    unittest.main()
