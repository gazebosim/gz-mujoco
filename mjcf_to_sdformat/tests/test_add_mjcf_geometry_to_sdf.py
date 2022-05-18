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

import sdformat as sdf
from ignition.math import Vector3d
from dm_control import mjcf

from mjcf_to_sdformat.converters import geometry as geometry_conv

import sdformat_mjcf.sdf_utils as su

GeometryType = sdf.Geometry.GeometryType


class GeometryTest(unittest.TestCase):

    def test_box(self):
        x_size, y_size, z_size = 1, 2, 3

        mujoco = mjcf.RootElement(model="test")
        body = mujoco.worldbody.add('body')
        geom = body.add('geom', type="box", name="box",
                        size=[x_size, y_size, z_size])
        sdf_geom = geometry_conv.add_mjcf_geometry_to_sdf(geom)

        self.assertEqual(GeometryType.BOX, sdf_geom.type())
        self.assertNotEqual(None, sdf_geom.box_shape())
        self.assertEqual(Vector3d(x_size, y_size, z_size) * 2,
                         sdf_geom.box_shape().size())

    def test_capsule(self):
        radius = 5.
        length = 20.

        mujoco = mjcf.RootElement(model="test")
        body = mujoco.worldbody.add('body')
        geom = body.add('geom', type="capsule", size=[radius, length])
        sdf_geom = geometry_conv.add_mjcf_geometry_to_sdf(geom)

        self.assertEqual(GeometryType.CAPSULE, sdf_geom.type())
        self.assertNotEqual(None, sdf_geom.capsule_shape())
        self.assertEqual(radius, sdf_geom.capsule_shape().radius())
        self.assertEqual(length, sdf_geom.capsule_shape().length() / 2)

    def test_capsule_fromto(self):
        radius = 5.
        length = 1.

        mujoco = mjcf.RootElement(model="test")
        body = mujoco.worldbody.add('body')
        geom = body.add('geom', type="capsule", size=[radius],
                        fromto=[0, 0, 0, 0, 0, 1])
        sdf_geom = geometry_conv.add_mjcf_geometry_to_sdf(geom)

        self.assertEqual(GeometryType.CAPSULE, sdf_geom.type())
        self.assertNotEqual(None, sdf_geom.capsule_shape())
        self.assertEqual(radius, sdf_geom.capsule_shape().radius())
        self.assertEqual(length, sdf_geom.capsule_shape().length())

    def test_cylinder(self):
        radius = 5.
        length = 20.

        mujoco = mjcf.RootElement(model="test")
        body = mujoco.worldbody.add('body')
        geom = body.add('geom', type="cylinder", size=[radius, length])
        sdf_geom = geometry_conv.add_mjcf_geometry_to_sdf(geom)

        self.assertEqual(GeometryType.CYLINDER, sdf_geom.type())
        self.assertNotEqual(None, sdf_geom.cylinder_shape())
        self.assertEqual(radius, sdf_geom.cylinder_shape().radius())
        self.assertEqual(length, sdf_geom.cylinder_shape().length() / 2)

    def test_cylinder_fromto(self):
        radius = 5.
        length = 1.

        mujoco = mjcf.RootElement(model="test")
        body = mujoco.worldbody.add('body')
        geom = body.add('geom', type="cylinder", size=[radius],
                        fromto=[0, 0, 0, 0, 0, 1])
        sdf_geom = geometry_conv.add_mjcf_geometry_to_sdf(geom)

        self.assertEqual(GeometryType.CYLINDER, sdf_geom.type())
        self.assertNotEqual(None, sdf_geom.cylinder_shape())
        self.assertEqual(radius, sdf_geom.cylinder_shape().radius())
        self.assertEqual(length, sdf_geom.cylinder_shape().length())

    def test_ellipsoid(self):
        x_radius = 1.
        y_radius = 2.
        z_radius = 3.
        mujoco = mjcf.RootElement(model="test")
        body = mujoco.worldbody.add('body')
        geom = body.add('geom',
                        type="ellipsoid",
                        size=[x_radius, y_radius, z_radius])
        sdf_geom = geometry_conv.add_mjcf_geometry_to_sdf(geom)

        self.assertEqual(GeometryType.ELLIPSOID, sdf_geom.type())
        self.assertNotEqual(None, sdf_geom.ellipsoid_shape())
        assert_allclose([x_radius, y_radius, z_radius],
                        su.vec3d_to_list(sdf_geom.ellipsoid_shape().radii()))

    def test_heightmap(self):
        pass

    def test_mesh(self):
        pass

    def test_plane(self):
        x_size = 5.
        y_size = 10.

        mujoco = mjcf.RootElement(model="test")
        body = mujoco.worldbody.add('body')
        geom = body.add('geom',
                        type="plane",
                        size=[x_size, y_size, 0.05])
        sdf_geom = geometry_conv.add_mjcf_geometry_to_sdf(geom)

        self.assertEqual(GeometryType.PLANE, sdf_geom.type())
        self.assertNotEqual(None, sdf_geom.plane_shape())
        assert_allclose([x_size * 2, y_size * 2],
                        su.vec2d_to_list(sdf_geom.plane_shape().size()))

    def test_sphere(self):
        radius = 5.

        mujoco = mjcf.RootElement(model="test")
        body = mujoco.worldbody.add('body')
        geom = body.add('geom',
                        type="sphere",
                        size=[radius])
        sdf_geom = geometry_conv.add_mjcf_geometry_to_sdf(geom)

        self.assertEqual(GeometryType.SPHERE, sdf_geom.type())
        self.assertNotEqual(None, sdf_geom.sphere_shape())
        self.assertEqual(radius, sdf_geom.sphere_shape().radius())


class VisualTest(unittest.TestCase):
    def test_basic_visual_attributes(self):
        radius = 5.

        mujoco = mjcf.RootElement(model="test")
        body = mujoco.worldbody.add('body')
        geom = body.add('geom',
                        type="sphere",
                        size=[radius])
        sdf_visual = geometry_conv.add_mjcf_visual_to_sdf(geom)

        self.assertEqual(GeometryType.SPHERE, sdf_visual.geometry().type())
        self.assertNotEqual(None, sdf_visual.geometry())

        sdf_geom = sdf_visual.geometry()
        self.assertEqual(GeometryType.SPHERE, sdf_geom.type())
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
        sdf_col = geometry_conv.add_mjcf_collision_to_sdf(geom)

        self.assertEqual(GeometryType.SPHERE, sdf_col.geometry().type())
        self.assertNotEqual(None, sdf_col.geometry())

        sdf_geom = sdf_col.geometry()
        self.assertEqual(GeometryType.SPHERE, sdf_geom.type())
        self.assertNotEqual(None, sdf_geom.sphere_shape())
        self.assertEqual(radius, sdf_geom.sphere_shape().radius())


if __name__ == "__main__":
    unittest.main()
