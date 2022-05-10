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

from sdformat_mjcf.converters.material import add_material


class MaterialTest(unittest.TestCase):

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

        material = add_material(geom, material)
        self.assertNotEqual(material, None)

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

        material = add_material(geom, material)
        self.assertNotEqual(material, None)

        self.assertAlmostEqual(0.8, material.specular)
        self.assertAlmostEqual(0.7, material.emission)
        self.assertEqual(1, material.rgba[0])
        self.assertEqual(0, material.rgba[1])
        self.assertEqual(0, material.rgba[2])
        self.assertEqual(1, material.rgba[3])

if __name__ == "__main__":
    unittest.main()
