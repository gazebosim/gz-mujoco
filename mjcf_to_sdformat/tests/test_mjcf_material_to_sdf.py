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

import unittest

from ignition.math import Color

from dm_control import mjcf
from dm_control import mujoco

from mjcf_to_sdformat.converters.material import mjcf_material_to_sdf
from mjcf_to_sdformat.converters.world import mjcf_worldbody_to_sdf

import sdformat as sdf

from tests.helpers import TEST_RESOURCES_DIR


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
        self.assertEqual(Color(0, 0.9, 0, 1.0), material.emissive())

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
        self.assertEqual(Color(0, 0, 0, 1.0), material.ambient())
        self.assertEqual(Color(1, 1, 1, 1.0), material.specular())
        self.assertEqual(Color(0, 0, 0, 1.0), material.emissive())
        pbr = material.pbr_material()
        self.assertNotEqual(None, pbr)
        workflow = pbr.workflow(sdf.PbrWorkflowType.METAL)
        self.assertNotEqual(None, workflow)
        self.assertEqual("tennis_ball.png", workflow.albedo_map())


if __name__ == "__main__":
    unittest.main()
