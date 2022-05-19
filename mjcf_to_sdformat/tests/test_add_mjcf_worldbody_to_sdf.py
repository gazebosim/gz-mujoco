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
import os

import sdformat as sdf

from mjcf_to_sdformat.converters.model import add_mjcf_worldbody_to_sdf
import sdformat_mjcf_utils.sdf_utils as su


class ModelTest(unittest.TestCase):

    def test_add_model(self):
        mjcf_model = mjcf.from_path(os.path.join(
                                    os.path.dirname(os.path.abspath(__file__)),
                                    "resources/test_mujoco.xml"))
        world = sdf.World()
        world.set_name("default")

        add_mjcf_worldbody_to_sdf(mjcf_model, world)

        self.assertEqual("default", world.name())
        self.assertEqual(1, world.model_count())
        model = world.model_by_index(0)
        self.assertNotEqual(None, model)
        self.assertEqual(3, model.link_count())
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

        link_2 = model.link_by_index(1)
        self.assertEqual("link_1", link_2.name())
        self.assertEqual(1, link_2.visual_count())
        self.assertEqual(1, link_2.collision_count())
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

        link_3 = model.link_by_index(2)
        self.assertEqual("body2", link_3.name())
        self.assertEqual(1, link_3.visual_count())
        self.assertEqual(1, link_3.collision_count())

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


if __name__ == "__main__":
    unittest.main()
