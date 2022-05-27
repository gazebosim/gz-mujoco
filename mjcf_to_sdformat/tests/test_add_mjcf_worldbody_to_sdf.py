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
from numpy.testing import assert_allclose
from dm_control import mjcf

from ignition.math import Color, Vector3d

import sdformat as sdf

from mjcf_to_sdformat.converters.world import mjcf_worldbody_to_sdf
import sdformat_mjcf_utils.sdf_utils as su

from tests.helpers import TEST_RESOURCES_DIR


class ModelTest(unittest.TestCase):

    def test_add_model(self):
        mjcf_model = mjcf.from_path(
            str(TEST_RESOURCES_DIR / "test_mujoco.xml"))

        world = sdf.World()
        world.set_name("default")

        mjcf_worldbody_to_sdf(mjcf_model, world)

        self.assertEqual("default", world.name())
        self.assertEqual(2, world.model_count())
        model = world.model_by_index(0)
        self.assertNotEqual(None, model)
        self.assertEqual(1, model.link_count())
        self.assertTrue(model.static())
        link_1 = model.link_by_index(0)
        self.assertEqual("link_0", link_1.name())
        self.assertEqual(1, link_1.visual_count())
        self.assertEqual(1, link_1.collision_count())
        assert_allclose([0, 0, 0], su.vec3d_to_list(link_1.raw_pose().pos()))
        assert_allclose([0, 0, 0],
                        su.vec3d_to_list(link_1.raw_pose().rot().euler()))
        visual_1 = link_1.visual_by_index(0)
        self.assertNotEqual(None, visual_1)
        self.assertEqual("visual_0", visual_1.name())
        assert_allclose([0, 0, 0], su.vec3d_to_list(visual_1.raw_pose().pos()))
        assert_allclose([0, 0, 0],
                        su.vec3d_to_list(visual_1.raw_pose().rot().euler()))

        collision_1 = link_1.collision_by_index(0)
        self.assertNotEqual(None, collision_1)
        self.assertEqual("collision_0", collision_1.name())
        assert_allclose([0, 0, 0],
                        su.vec3d_to_list(collision_1.raw_pose().pos()))
        assert_allclose([0, 0, 0],
                        su.vec3d_to_list(collision_1.raw_pose().rot().euler()))

        model = world.model_by_index(1)
        self.assertNotEqual(None, model)
        link_2 = model.link_by_index(0)
        self.assertEqual("link_1", link_2.name())
        self.assertEqual(2, link_2.visual_count())
        self.assertEqual(2, link_2.collision_count())
        assert_allclose([0, 0, 1], su.vec3d_to_list(link_2.raw_pose().pos()))
        assert_allclose([0, 0, 0],
                        su.vec3d_to_list(link_2.raw_pose().rot().euler()))
        mass_matrix = link_2.inertial().mass_matrix()
        self.assertEqual(2, mass_matrix.mass())
        self.assertEqual([0, 1, 0],
                         su.vec3d_to_list(link_2.inertial().pose().pos()))
        self.assertEqual([0, 0, 0],
                         su.vec3d_to_list(link_2.inertial().pose().rot()))
        assert_allclose([2, 2, 2],
                        su.vec3d_to_list(mass_matrix.diagonal_moments()))
        assert_allclose([0, 0, 0],
                        su.vec3d_to_list(mass_matrix.off_diagonal_moments()))

        visual_2 = link_2.visual_by_index(0)
        self.assertNotEqual(None, visual_2)
        self.assertEqual("visual_0", visual_2.name())
        assert_allclose([0.1, 0.3, 0.2],
                        su.vec3d_to_list(visual_2.raw_pose().pos()))
        assert_allclose([0, 0, 0],
                        su.vec3d_to_list(visual_2.raw_pose().rot().euler()))

        collision_2 = link_2.collision_by_index(0)
        self.assertNotEqual(None, collision_2)
        self.assertEqual("collision_0", collision_2.name())
        assert_allclose([0.1, 0.3, 0.2],
                        su.vec3d_to_list(collision_2.raw_pose().pos()))
        assert_allclose([0, 0, 0],
                        su.vec3d_to_list(collision_2.raw_pose().rot().euler()))

        visual_2 = link_2.visual_by_index(1)
        self.assertNotEqual(None, visual_2)
        self.assertEqual("visual_cylinder", visual_2.name())
        assert_allclose([0.0, 0.0, 0.0],
                        su.vec3d_to_list(visual_2.raw_pose().pos()))
        assert_allclose([1.570796, 0, 0],
                        su.vec3d_to_list(visual_2.raw_pose().rot().euler()),
                        rtol=1e-5)

        collision_2 = link_2.collision_by_index(1)
        self.assertNotEqual(None, collision_2)
        self.assertEqual("collision_cylinder", collision_2.name())
        assert_allclose([0.0, 0.0, 0.0],
                        su.vec3d_to_list(collision_2.raw_pose().pos()))
        assert_allclose([1.570796, 0, 0],
                        su.vec3d_to_list(collision_2.raw_pose().rot().euler()),
                        rtol=1e-5)

        link_3 = model.link_by_index(1)
        self.assertEqual("body2", link_3.name())
        self.assertEqual(1, link_3.visual_count())
        self.assertEqual(1, link_3.collision_count())

        self.assertEqual(1, link_3.light_count())
        light_link = link_3.light_by_index(0)
        self.assertNotEqual(None, light_link)
        self.assertEqual(0, light_link.constant_attenuation_factor())
        self.assertEqual(0.01, light_link.linear_attenuation_factor())
        self.assertEqual(0.001, light_link.quadratic_attenuation_factor())
        self.assertTrue(light_link.cast_shadows())
        self.assertEqual(Color(0.8, 0.8, 0.8), light_link.diffuse())
        self.assertEqual(Color(0.2, 0.2, 0.2), light_link.specular())
        self.assertEqual(sdf.LightType.DIRECTIONAL, light_link.type())
        self.assertEqual(Vector3d(0.5, 0.2, 0.0), light_link.direction())
        assert_allclose([0, 0, 10],
                        su.vec3d_to_list(light_link.raw_pose().pos()))
        assert_allclose([0, 0, 0],
                        su.vec3d_to_list(light_link.raw_pose().rot().euler()))

        mass_matrix = link_3.inertial().mass_matrix()
        self.assertEqual(2, mass_matrix.mass())
        self.assertEqual([1, 2, 3],
                         su.vec3d_to_list(link_3.inertial().pose().pos()))
        self.assertEqual([0, 0, 0],
                         su.vec3d_to_list(link_3.inertial().pose().rot()))
        assert_allclose([0.026026, 0.023326, 0.011595],
                        su.vec3d_to_list(mass_matrix.diagonal_moments()))
        assert_allclose([0, 0, 0],
                        su.vec3d_to_list(mass_matrix.off_diagonal_moments()))

        assert_allclose([0, 1, 0], su.vec3d_to_list(link_3.raw_pose().pos()))
        assert_allclose([0, 0, 0],
                        su.vec3d_to_list(link_3.raw_pose().rot().euler()))
        visual_3 = link_3.visual_by_index(0)
        self.assertNotEqual(None, visual_3)
        self.assertEqual("visual_visual_body2", visual_3.name())
        assert_allclose([0.0, 0, 0],
                        su.vec3d_to_list(visual_3.raw_pose().pos()))
        assert_allclose([0, 0, 0],
                        su.vec3d_to_list(visual_3.raw_pose().rot().euler()))

        collision_3 = link_3.collision_by_index(0)
        self.assertNotEqual(None, collision_3)
        self.assertEqual("collision_col_body2", collision_3.name())
        assert_allclose([0, 0, 0],
                        su.vec3d_to_list(collision_3.raw_pose().pos()))
        assert_allclose([0, 0, 0],
                        su.vec3d_to_list(collision_3.raw_pose().rot().euler()))

        link_4 = model.link_by_index(2)
        self.assertEqual("body3", link_4.name())
        self.assertEqual(1, link_4.visual_count())
        self.assertEqual(1, link_4.collision_count())
        self.assertEqual("body2", link_4.pose_relative_to())

        mass_matrix = link_4.inertial().mass_matrix()
        self.assertEqual(1, mass_matrix.mass())
        self.assertEqual([0, 0, 0],
                         su.vec3d_to_list(link_4.inertial().pose().pos()))
        self.assertEqual([0, 0, 0],
                         su.vec3d_to_list(link_4.inertial().pose().rot()))
        assert_allclose([1, 1, 1],
                        su.vec3d_to_list(mass_matrix.diagonal_moments()))
        assert_allclose([0, 0, 0],
                        su.vec3d_to_list(mass_matrix.off_diagonal_moments()))

        assert_allclose([0, 0, 1], su.vec3d_to_list(link_4.raw_pose().pos()))
        assert_allclose([0, 0, 0],
                        su.vec3d_to_list(link_4.raw_pose().rot().euler()))
        visual_4 = link_4.visual_by_index(0)
        self.assertNotEqual(None, visual_4)
        self.assertEqual("visual_visual_body3", visual_4.name())
        assert_allclose([0.0, 0, 0],
                        su.vec3d_to_list(visual_4.raw_pose().pos()))
        assert_allclose([0, 0, 0],
                        su.vec3d_to_list(visual_4.raw_pose().rot().euler()))

        collision_4 = link_4.collision_by_index(0)
        self.assertNotEqual(None, collision_4)
        self.assertEqual("collision_visual_body3", collision_4.name())
        assert_allclose([0, 0, 0],
                        su.vec3d_to_list(collision_4.raw_pose().pos()))
        assert_allclose([0, 0, 0],
                        su.vec3d_to_list(collision_4.raw_pose().rot().euler()))

        self.assertEqual(1, world.light_count())
        light_1 = world.light_by_index(0)
        self.assertNotEqual(None, light_1)
        self.assertEqual(0, light_1.constant_attenuation_factor())
        self.assertEqual(0.01, light_1.linear_attenuation_factor())
        self.assertEqual(0.001, light_1.quadratic_attenuation_factor())
        self.assertFalse(light_1.cast_shadows())
        self.assertEqual(Color(0.5, 0.5, 0.5), light_1.diffuse())
        self.assertEqual(Color(0.2, 0.2, 0.2), light_1.specular())
        self.assertEqual(sdf.LightType.SPOT, light_1.type())
        self.assertEqual(Vector3d(0, 0, -1), light_1.direction())
        assert_allclose([0, 0, 3],
                        su.vec3d_to_list(light_1.raw_pose().pos()))
        assert_allclose([0, 0, 0],
                        su.vec3d_to_list(light_1.raw_pose().rot().euler()))


if __name__ == "__main__":
    unittest.main()
