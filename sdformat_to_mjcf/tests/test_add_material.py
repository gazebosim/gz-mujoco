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
import os

from dm_control import mjcf

from ignition.math import Color

import sdformat as sdf

from sdformat_to_mjcf.converters.material import add_material
from tests import helpers


class MaterialTest(helpers.TestCase):

    def test_material_pbr(self):
        pbr = sdf.Pbr()
        workflow = sdf.PbrWorkflow()
        workflow.set_type(sdf.PbrWorkflow.PbrWorkflowType.METAL)
        workflow.set_albedo_map(os.path.join(
                                os.path.dirname(os.path.abspath(__file__)),
                                "resources/box_obj/textures/albedo_map.png"))
        pbr.set_workflow(workflow.type(), workflow)

        material = sdf.Material()
        material.set_pbr_material(pbr)

        mujoco = mjcf.RootElement(model="test")
        body = mujoco.worldbody.add('body')
        geom = body.add(
            "geom",
            name="geometry_test")

        material_mjcf = add_material(geom, material)
        self.assertNotEqual(None, material_mjcf)
        self.assertEqual("material_albedo_map", material_mjcf.name)
        self.assertEqual("albedo_map", material_mjcf.texture.name)
        self.assertEqual("2d", material_mjcf.texture.type)
        self.assertEqual(1, material_mjcf.texture.gridsize[0])
        self.assertEqual(1, material_mjcf.texture.gridsize[1])
        self.assertEqual(1, material_mjcf.texrepeat[0])
        self.assertEqual(1, material_mjcf.texrepeat[1])
        self.assertEqual("true", material_mjcf.texuniform)
        self.assertEqual(0.0, material_mjcf.emission)
        self.assertEqual(0.0, material_mjcf.specular)

        geom2 = body.add(
            "geom",
            name="geometry_test2")
        material_mjcf2 = add_material(geom2, material)
        self.assertEqual(material_mjcf2, material_mjcf)

    def test_material_pbr_bad_extension(self):
        pbr = sdf.Pbr()
        workflow = sdf.PbrWorkflow()
        workflow.set_type(sdf.PbrWorkflow.PbrWorkflowType.METAL)
        workflow.set_albedo_map(os.path.join(
                                os.path.dirname(os.path.abspath(__file__)),
                                "resources/box_obj/textures/albedo_map"))
        pbr.set_workflow(workflow.type(), workflow)

        material = sdf.Material()
        material.set_pbr_material(pbr)

        mujoco = mjcf.RootElement(model="test")
        body = mujoco.worldbody.add('body')
        geom = body.add(
            "geom",
            name="geometry_test")

        self.assertRaises(RuntimeError, add_material, geom, material)

    def test_material_color(self):
        material = sdf.Material()
        material.set_diffuse(Color(1, 0, 0))
        material.set_ambient(Color(0, 1, 0))
        material.set_specular(Color(0.8, 0.8, 0.8))
        material.set_emissive(Color(0.7, 0.7, 0.7))

        mujoco = mjcf.RootElement(model="test")
        body = mujoco.worldbody.add('body')
        geom = body.add(
            "geom",
            name="geometry_test")

        material_mjcf = add_material(geom, material)
        self.assertNotEqual(material_mjcf, None)

        self.assertAlmostEqual(0.8, material_mjcf.specular)
        self.assertAlmostEqual(0.7, material_mjcf.emission)
        self.assertAlmostEqual(0.8, material_mjcf.rgba[0])
        self.assertAlmostEqual(0.4, material_mjcf.rgba[1])
        self.assertAlmostEqual(0, material_mjcf.rgba[2])
        self.assertAlmostEqual(1, material_mjcf.rgba[3])


if __name__ == "__main__":
    unittest.main()
