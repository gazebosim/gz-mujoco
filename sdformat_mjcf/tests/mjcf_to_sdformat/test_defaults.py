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

from sdformat_mjcf.mjcf_to_sdformat.converters.world import (
    mjcf_worldbody_to_sdf,
)

import sdformat as sdf

from tests.helpers import get_resources_dir

TEST_RESOURCES_DIR = get_resources_dir()


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
        self.assertNotEqual(None, link)
        self.assertEqual(sdf.GeometryType.CAPSULE, visual.geometry().type())
        capsule_shape2 = visual.geometry().capsule_shape()
        self.assertNotEqual(None, capsule_shape2)
        self.assertEqual(0.6, capsule_shape2.length())
        self.assertEqual(0.07, capsule_shape2.radius())

    def test_inheritance(self):
        filename = str(TEST_RESOURCES_DIR / "test_defaults_inheritance.xml")
        mjcf_model = mjcf.from_path(filename)
        self.assertIsNotNone(mjcf_model)
        physics = mjcf.Physics.from_xml_path(filename)
        self.assertIsNotNone(physics)

        world = sdf.World()
        world.set_name("default")

        mjcf_worldbody_to_sdf(mjcf_model, physics, world)
        root = sdf.Root()
        root.add_world(world)
        model = world.model_by_index(1)
        self.assertNotEqual(None, model)
        self.assertEqual(2, model.link_count())

        link1 = model.link_by_name("body1")
        self.assertIsNotNone(link1)

        # shape1's class is "sub" based on the parent body's childclass
        visual_shape1 = link1.visual_by_name("visual_shape1")
        self.assertEqual(sdf.GeometryType.CYLINDER,
                         visual_shape1.geometry().type())
        shape1_cylinder = visual_shape1.geometry().cylinder_shape()
        self.assertEqual(6, shape1_cylinder.length())
        self.assertEqual(2, shape1_cylinder.radius())

        # shape2's class is "main" based shape2's own class regardles of its
        # parent body's childclass.
        visual_shape2 = link1.visual_by_name("visual_shape2")
        self.assertEqual(sdf.GeometryType.SPHERE,
                         visual_shape2.geometry().type())
        shape2_sphere = visual_shape2.geometry().sphere_shape()
        self.assertEqual(2, shape2_sphere.radius())

        # shape3's class is "small_shape" based shape3's own class regardless
        # of its parent body's childclass.
        visual_shape3 = link1.visual_by_name("visual_shape3")
        self.assertEqual(sdf.GeometryType.SPHERE,
                         visual_shape3.geometry().type())
        shape3_sphere = visual_shape3.geometry().sphere_shape()
        self.assertEqual(0.5, shape3_sphere.radius())

        link2 = model.link_by_name("body2")
        self.assertIsNotNone(link2)

        # shape4's class is "main" since it doesn't have a parent body with a
        # childclass nor its own class.
        visual_shape4 = link2.visual_by_name("visual_shape4")
        self.assertEqual(sdf.GeometryType.SPHERE,
                         visual_shape4.geometry().type())
        shape4_cylinder = visual_shape4.geometry().sphere_shape()
        self.assertEqual(2, shape4_cylinder.radius())

        # shape5's class is "main" based shape5's own class.
        visual_shape5 = link2.visual_by_name("visual_shape5")
        self.assertEqual(sdf.GeometryType.SPHERE,
                         visual_shape5.geometry().type())
        shape5_sphere = visual_shape5.geometry().sphere_shape()
        self.assertEqual(2, shape5_sphere.radius())

        # shape3's class is "small_shape" based shape3's own class.
        visual_shape6 = link2.visual_by_name("visual_shape6")
        self.assertEqual(sdf.GeometryType.SPHERE,
                         visual_shape6.geometry().type())
        shape6_sphere = visual_shape6.geometry().sphere_shape()
        self.assertEqual(0.5, shape6_sphere.radius())

    def test_default_friction(self):
        filename = str(TEST_RESOURCES_DIR / "tennis_ball.xml")
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
        self.assertEqual(1, model.link_count())
        link = model.link_by_index(0)
        self.assertNotEqual(None, link)

        collision = link.collision_by_index(0)
        self.assertEqual(sdf.GeometryType.SPHERE, collision.geometry().type())
        sphere_shape = collision.geometry().sphere_shape()
        self.assertNotEqual(None, sphere_shape)
        self.assertEqual(0.03, sphere_shape.radius())
        surface = collision.surface()
        self.assertEqual(0.5, surface.friction().ode().mu())


if __name__ == "__main__":
    unittest.main()
