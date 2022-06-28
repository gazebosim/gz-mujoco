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
from numpy.testing import assert_allclose

import sdformat as sdf
from ignition.math import Vector3d
from dm_control import mjcf

from sdformat_mjcf.mjcf_to_sdformat.converters import geometry as geometry_conv

import sdformat_mjcf.utils.sdf_utils as su
from tests.helpers import get_resources_dir

TEST_RESOURCES_DIR = get_resources_dir()


class GeometryTest(unittest.TestCase):

    def setUp(self):
        self.mujoco = mjcf.RootElement(model="test")
        self.body = self.mujoco.worldbody.add('body')

    def test_box(self):
        x_size, y_size, z_size = 1, 2, 3
        geom = self.body.add('geom', type="box", name="box",
                             size=[x_size, y_size, z_size])
        sdf_geom = geometry_conv.mjcf_geom_to_sdf(geom)

        self.assertEqual(sdf.GeometryType.BOX, sdf_geom.type())
        self.assertNotEqual(None, sdf_geom.box_shape())
        self.assertEqual(Vector3d(x_size, y_size, z_size) * 2,
                         sdf_geom.box_shape().size())

    def test_box_fromto(self):
        geom = self.body.add('geom', type="box", name="box",
                             fromto=[1, 0.5, 1, -1, -0.5, -1])

        with self.assertRaises(RuntimeError):
            geometry_conv.mjcf_geom_to_sdf(geom)

    def test_capsule(self):
        radius = 5.
        length = 20.

        geom = self.body.add('geom', type="capsule", size=[radius, length])
        sdf_geom = geometry_conv.mjcf_geom_to_sdf(geom)

        self.assertEqual(sdf.GeometryType.CAPSULE, sdf_geom.type())
        self.assertNotEqual(None, sdf_geom.capsule_shape())
        self.assertEqual(radius, sdf_geom.capsule_shape().radius())
        self.assertEqual(length, sdf_geom.capsule_shape().length() / 2)

    def test_capsule_fromto(self):
        radius = 5.
        length = 1.

        geom = self.body.add('geom', type="capsule", size=[radius],
                             fromto=[0, 0, 0, 0, 0, 1])
        sdf_geom = geometry_conv.mjcf_geom_to_sdf(geom)

        self.assertEqual(sdf.GeometryType.CAPSULE, sdf_geom.type())
        self.assertNotEqual(None, sdf_geom.capsule_shape())
        self.assertEqual(radius, sdf_geom.capsule_shape().radius())
        self.assertEqual(length, sdf_geom.capsule_shape().length())

    def test_cylinder(self):
        radius = 5.
        length = 20.

        geom = self.body.add('geom', type="cylinder", size=[radius, length])
        sdf_geom = geometry_conv.mjcf_geom_to_sdf(geom)

        self.assertEqual(sdf.GeometryType.CYLINDER, sdf_geom.type())
        self.assertNotEqual(None, sdf_geom.cylinder_shape())
        self.assertEqual(radius, sdf_geom.cylinder_shape().radius())
        self.assertEqual(length, sdf_geom.cylinder_shape().length() / 2)

    def test_cylinder_fromto(self):
        radius = 5.
        length = 1.

        geom = self.body.add('geom', type="cylinder", size=[radius],
                             fromto=[0, 0, 0, 0, 0, 1])
        sdf_geom = geometry_conv.mjcf_geom_to_sdf(geom)

        self.assertEqual(sdf.GeometryType.CYLINDER, sdf_geom.type())
        self.assertNotEqual(None, sdf_geom.cylinder_shape())
        self.assertEqual(radius, sdf_geom.cylinder_shape().radius())
        self.assertEqual(length, sdf_geom.cylinder_shape().length())

    def test_ellipsoid(self):
        x_radius = 1.
        y_radius = 2.
        z_radius = 3.
        geom = self.body.add('geom',
                             type="ellipsoid",
                             size=[x_radius, y_radius, z_radius])
        sdf_geom = geometry_conv.mjcf_geom_to_sdf(geom)

        self.assertEqual(sdf.GeometryType.ELLIPSOID, sdf_geom.type())
        self.assertNotEqual(None, sdf_geom.ellipsoid_shape())
        assert_allclose([x_radius, y_radius, z_radius],
                        su.vec3d_to_list(sdf_geom.ellipsoid_shape().radii()))

    def test_ellipsoid_fromto(self):
        geom = self.body.add('geom', type="ellipsoid",
                             fromto=[1, 0.5, 2, -1, -0.5, -2])

        with self.assertRaises(RuntimeError):
            geometry_conv.mjcf_geom_to_sdf(geom)

    def test_heightmap(self):
        pass

    def test_mesh(self):
        filename = str(TEST_RESOURCES_DIR / "mug.xml")
        mjcf_model = mjcf.from_path(filename)
        self.assertIsNotNone(mjcf_model)
        geom = mjcf_model.find("geom", "mug")
        self.assertIsNotNone(geom)
        sdf_geom = geometry_conv.mjcf_geom_to_sdf(geom)
        self.assertEqual(sdf.GeometryType.MESH, sdf_geom.type())
        mesh_shape = sdf_geom.mesh_shape()
        self.assertIsNotNone(mesh_shape)
        self.assertEqual(Vector3d(0.01, 0.01, 0.01), mesh_shape.scale())

    def test_mesh_without_type(self):
        test_mjcf = f"""
        <mujoco model="mug">
            <asset>
                <mesh file="{TEST_RESOURCES_DIR / "mug.obj"}"/>
            </asset>
            <worldbody>
                <geom name="mug" mesh="mug"/>
            </worldbody>
        </mujoco>
        """
        mjcf_model = mjcf.from_xml_string(test_mjcf)
        self.assertIsNotNone(mjcf_model)
        geom = mjcf_model.find("geom", "mug")
        self.assertIsNotNone(geom)
        sdf_geom = geometry_conv.mjcf_geom_to_sdf(geom)
        self.assertEqual(sdf.GeometryType.MESH, sdf_geom.type())
        mesh_shape = sdf_geom.mesh_shape()
        self.assertIsNotNone(mesh_shape)

    def test_plane(self):
        x_size = 5.
        y_size = 10.

        geom = self.body.add('geom', type="plane", size=[x_size, y_size, 0.05])
        sdf_geom = geometry_conv.mjcf_geom_to_sdf(geom)

        self.assertEqual(sdf.GeometryType.PLANE, sdf_geom.type())
        self.assertNotEqual(None, sdf_geom.plane_shape())
        assert_allclose([x_size * 2, y_size * 2],
                        su.vec2d_to_list(sdf_geom.plane_shape().size()))

    def test_infinite_plane(self):
        mujoco = mjcf.RootElement(model="test")
        body = mujoco.worldbody.add('body')
        geom = body.add('geom', type="plane", size=[0, 0, 1])
        sdf_geom = geometry_conv.mjcf_geom_to_sdf(geom)

        self.assertEqual(sdf.GeometryType.PLANE, sdf_geom.type())
        self.assertNotEqual(None, sdf_geom.plane_shape())
        assert_allclose([1e6, 1e6],
                        su.vec2d_to_list(sdf_geom.plane_shape().size()))

    def test_sphere(self):
        radius = 5.

        geom = self.body.add('geom', type="sphere", size=[radius])
        sdf_geom = geometry_conv.mjcf_geom_to_sdf(geom)

        self.assertEqual(sdf.GeometryType.SPHERE, sdf_geom.type())
        self.assertNotEqual(None, sdf_geom.sphere_shape())
        self.assertEqual(radius, sdf_geom.sphere_shape().radius())

    def test_no_type(self):
        radius = 5.

        geom = self.body.add('geom', size=[radius])
        sdf_geom = geometry_conv.mjcf_geom_to_sdf(geom)

        self.assertEqual(sdf.GeometryType.SPHERE, sdf_geom.type())
        self.assertNotEqual(None, sdf_geom.sphere_shape())
        self.assertEqual(radius, sdf_geom.sphere_shape().radius())


class SurfaceTest(unittest.TestCase):
    def test_friction(self):
        mjcf_model = mjcf.from_path(
            str(TEST_RESOURCES_DIR / "test_mujoco.xml"))

        sdf_plane_collision = geometry_conv.mjcf_collision_to_sdf(
            mjcf_model.worldbody.geom[0])
        self.assertIsNotNone(sdf_plane_collision)
        self.assertEqual(sdf.GeometryType.PLANE,
            sdf_plane_collision.geom().type())
        self.assertEqual(0.9,
            sdf_plane_collision.surface().friction().ode().mu())

        sdf_rail1_collision = geometry_conv.mjcf_collision_to_sdf(
            mjcf_model.worldbody.geom[1])
        self.assertIsNotNone(sdf_rail1_collision)
        self.assertEqual(sdf.GeometryType.CAPSULE,
            sdf_rail1_collision.geom().type())
        self.assertEqual(1.0,
            sdf_rail1_collision.surface().friction().ode().mu())


class VisualTest(unittest.TestCase):
    def test_basic_visual_attributes(self):
        radius = 5.

        mujoco = mjcf.RootElement(model="test")
        body = mujoco.worldbody.add('body')
        geom = body.add('geom',
                        type="sphere",
                        size=[radius])
        sdf_visual = geometry_conv.mjcf_visual_to_sdf(geom)

        self.assertEqual(sdf.GeometryType.SPHERE, sdf_visual.geometry().type())
        self.assertNotEqual(None, sdf_visual.geometry())

        sdf_geom = sdf_visual.geometry()
        self.assertEqual(sdf.GeometryType.SPHERE, sdf_geom.type())
        self.assertNotEqual(None, sdf_geom.sphere_shape())
        self.assertEqual(radius, sdf_geom.sphere_shape().radius())


class CollisionTest(unittest.TestCase):
    def test_basic_collision_attributes(self):
        radius = 5.

        mujoco = mjcf.RootElement(model="test")
        body = mujoco.worldbody.add('body')
        geom = body.add('geom',
                        type="sphere",
                        size=[radius])
        sdf_col = geometry_conv.mjcf_collision_to_sdf(geom)

        self.assertEqual(sdf.GeometryType.SPHERE, sdf_col.geometry().type())
        self.assertNotEqual(None, sdf_col.geometry())

        sdf_geom = sdf_col.geometry()
        self.assertEqual(sdf.GeometryType.SPHERE, sdf_geom.type())
        self.assertNotEqual(None, sdf_geom.sphere_shape())
        self.assertEqual(radius, sdf_geom.sphere_shape().radius())


if __name__ == "__main__":
    unittest.main()
