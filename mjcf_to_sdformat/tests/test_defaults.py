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

from dm_control import mjcf
from dm_control import mujoco

from mjcf_to_sdformat.converters.world import mjcf_worldbody_to_sdf

import sdformat as sdf

from tests.helpers import TEST_RESOURCES_DIR


class DefaultsTest(unittest.TestCase):

    def test_defaults(self):
        filename = str(TEST_RESOURCES_DIR / "test_defaults.xml")
        mjcf_model = mjcf.from_path(filename)
        physics = mujoco.Physics.from_xml_path(filename)

        world = sdf.World()
        world.set_name("default")

        mjcf_worldbody_to_sdf(mjcf_model, physics, world)

        self.assertEqual("default", world.name())
        self.assertEqual(2, world.model_count())
        model = world.model_by_index(0)
        self.assertNotEqual(None, model)
        self.assertEqual(1, model.link_count())
        self.assertTrue(model.static())

        model = world.model_by_index(1)
        self.assertNotEqual(None, model)
        link = model.link_by_index(0)
        self.assertNotEqual(None, link)

        visual = link.visual_by_index(0)
        self.assertEqual(sdf.GeometryType.CAPSULE, visual.geometry().type())
        capsule_shape = visual.geometry().capsule_shape()
        self.assertNotEqual(None, capsule_shape)
        self.assertEqual(0.6, capsule_shape.length())
        self.assertEqual(0.07, capsule_shape.radius())

        link = model.link_by_index(1)
        self.assertNotEqual(None, link)
        visual = link.visual_by_index(0)
        self.assertEqual(sdf.GeometryType.SPHERE, visual.geometry().type())
        sphere_shape = visual.geometry().sphere_shape()
        self.assertNotEqual(None, sphere_shape)
        self.assertEqual(0.01, sphere_shape.radius())


if __name__ == "__main__":
    unittest.main()
