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
import numpy as np
from numpy.testing import assert_allclose
from math import pi
import os

import sdformat as sdf
from ignition.math import Pose3d, Vector2d, Vector3d
from dm_control import mjcf

from tests import helpers
from sdformat_to_mjcf.converters import geometry as geometry_conv


class GeometryTest(helpers.TestCase):

    test_pose = Pose3d(1, 2, 3, pi / 2, pi / 3, pi / 4)
    expected_pos = [1., 2., 3.]
    expected_euler = [90., 60., 45.]

    def test_box(self):
        box = sdf.Box()
        x_size, y_size, z_size = 1, 2, 3
        box.set_size(Vector3d(x_size, y_size, z_size))
        geometry = sdf.Geometry()
        geometry.set_box_shape(box)
        geometry.set_type(sdf.GeometryType.BOX)

        mujoco = mjcf.RootElement(model="test")
        body = mujoco.worldbody.add('body')
        mj_geom = geometry_conv.add_geometry(body, "box_shape", self.test_pose,
                                             geometry)
        self.assertEqual("box_shape", mj_geom.name)
        self.assertEqual("box", mj_geom.type)
        assert_allclose([x_size / 2., y_size / 2., z_size / 2.], mj_geom.size)
        assert_allclose(self.expected_pos, mj_geom.pos)
        assert_allclose(self.expected_euler, mj_geom.euler)

    def test_capsule(self):
        capsule = sdf.Capsule()
        radius = 5.
        length = 20.
        capsule.set_radius(radius)
        capsule.set_length(length)

        geometry = sdf.Geometry()
        geometry.set_capsule_shape(capsule)
        geometry.set_type(sdf.GeometryType.CAPSULE)

        mujoco = mjcf.RootElement(model="test")
        body = mujoco.worldbody.add('body')
        mj_geom = geometry_conv.add_geometry(body, "capsule_shape",
                                             self.test_pose, geometry)
        self.assertEqual("capsule_shape", mj_geom.name)
        self.assertEqual("capsule", mj_geom.type)
        assert_allclose([radius, length / 2.], mj_geom.size)
        assert_allclose(self.expected_pos, mj_geom.pos)
        assert_allclose(self.expected_euler, mj_geom.euler)

    def test_cylinder(self):
        cylinder = sdf.Cylinder()
        radius = 5.
        length = 20.
        cylinder.set_radius(radius)
        cylinder.set_length(length)

        geometry = sdf.Geometry()
        geometry.set_cylinder_shape(cylinder)
        geometry.set_type(sdf.GeometryType.CYLINDER)

        mujoco = mjcf.RootElement(model="test")
        body = mujoco.worldbody.add('body')
        mj_geom = geometry_conv.add_geometry(body, "cylinder_shape",
                                             self.test_pose, geometry)
        self.assertEqual("cylinder_shape", mj_geom.name)
        self.assertEqual("cylinder", mj_geom.type)
        assert_allclose([radius, length / 2.], mj_geom.size)
        assert_allclose(self.expected_pos, mj_geom.pos)
        assert_allclose(self.expected_euler, mj_geom.euler)

    def test_ellipsoid(self):
        ellipsoid = sdf.Ellipsoid()
        x_radius = 1.
        y_radius = 2.
        z_radius = 3.
        ellipsoid.set_radii(Vector3d(x_radius, y_radius, z_radius))

        geometry = sdf.Geometry()
        geometry.set_ellipsoid_shape(ellipsoid)
        geometry.set_type(sdf.GeometryType.ELLIPSOID)

        mujoco = mjcf.RootElement(model="test")
        body = mujoco.worldbody.add('body')
        mj_geom = geometry_conv.add_geometry(body, "ellipsoid_shape",
                                             self.test_pose, geometry)
        self.assertEqual("ellipsoid_shape", mj_geom.name)
        self.assertEqual("ellipsoid", mj_geom.type)
        assert_allclose([x_radius, y_radius, z_radius], mj_geom.size)
        assert_allclose(self.expected_pos, mj_geom.pos)
        assert_allclose(self.expected_euler, mj_geom.euler)

    def test_heightmap(self):
        pass

    def test_mesh(self):
        mesh = sdf.Mesh()
        mesh.set_uri("meshes/box.obj")
        mesh.set_file_path(os.path.join(
                           os.path.dirname(os.path.abspath(__file__)),
                           "resources/box_obj/model.sdf"))

        geometry = sdf.Geometry()
        geometry.set_mesh_shape(mesh)
        geometry.set_type(sdf.GeometryType.MESH)

        mujoco = mjcf.RootElement(model="test")
        body = mujoco.worldbody.add('body')
        mj_geom = geometry_conv.add_geometry(body, "mesh_shape",
                                             self.test_pose, geometry)
        self.assertEqual("mesh_shape", mj_geom.name)
        self.assertEqual("mesh", mj_geom.type)
        self.assertEqual(1, len(mujoco.asset.find_all('mesh')))

    def test_plane(self):
        plane = sdf.Plane()
        x_size = 5.
        y_size = 10.
        normal = np.array([1, 2, 3])
        normal_unit = normal / np.linalg.norm(normal)
        plane.set_size(Vector2d(x_size, y_size))
        plane.set_normal(Vector3d(*normal_unit))

        geometry = sdf.Geometry()
        geometry.set_plane_shape(plane)
        geometry.set_type(sdf.GeometryType.PLANE)

        mujoco = mjcf.RootElement(model="test")
        body = mujoco.worldbody.add('body')
        mj_geom = geometry_conv.add_geometry(body, "plane_shape",
                                             self.test_pose, geometry)
        self.assertEqual("plane_shape", mj_geom.name)
        self.assertEqual("plane", mj_geom.type)
        assert_allclose([x_size / 2., y_size / 2., 0.], mj_geom.size)
        assert_allclose(self.expected_pos, mj_geom.pos)
        assert_allclose(self.expected_euler, mj_geom.euler)

    def test_sphere(self):
        sphere = sdf.Sphere()
        radius = 5.
        sphere.set_radius(radius)

        geometry = sdf.Geometry()
        geometry.set_sphere_shape(sphere)
        geometry.set_type(sdf.GeometryType.SPHERE)

        mujoco = mjcf.RootElement(model="test")
        body = mujoco.worldbody.add('body')
        mj_geom = geometry_conv.add_geometry(body, "sphere_shape",
                                             self.test_pose, geometry)
        self.assertEqual("sphere_shape", mj_geom.name)
        self.assertEqual("sphere", mj_geom.type)
        assert_allclose(radius, mj_geom.size)
        assert_allclose(self.expected_pos, mj_geom.pos)
        assert_allclose(self.expected_euler, mj_geom.euler)


class CollisionTest(helpers.TestCase):

    def test_basic_collision_attributes(self):
        collision = sdf.Collision()
        collision.set_name("c1")
        collision.set_raw_pose(Pose3d(1, 2, 3, pi / 2, pi / 3, pi / 4))

        geometry = sdf.Geometry()
        geometry.set_box_shape(sdf.Box())
        geometry.set_type(sdf.GeometryType.BOX)
        collision.set_geometry(geometry)

        mujoco = mjcf.RootElement(model="test")
        body = mujoco.worldbody.add('body')
        mj_geom = geometry_conv.add_collision(body, collision)
        self.assertEqual("c1", mj_geom.name)
        assert_allclose([1., 2., 3.], mj_geom.pos)
        assert_allclose([90., 60., 45.], mj_geom.euler)
        self.assertEqual(geometry_conv.COLLISION_GEOM_GROUP, mj_geom.group)


class VisualTest(helpers.TestCase):

    def test_basic_visual_attributes(self):
        visual = sdf.Visual()
        visual.set_name("v1")
        visual.set_raw_pose(Pose3d(1, 2, 3, pi / 2, pi / 3, pi / 4))

        geometry = sdf.Geometry()
        geometry.set_box_shape(sdf.Box())
        geometry.set_type(sdf.GeometryType.BOX)
        visual.set_geometry(geometry)

        mujoco = mjcf.RootElement(model="test")
        body = mujoco.worldbody.add('body')
        mj_geom = geometry_conv.add_visual(body, visual)
        self.assertEqual("v1", mj_geom.name)
        assert_allclose([1., 2., 3.], mj_geom.pos)
        assert_allclose([90., 60., 45.], mj_geom.euler)
        self.assertEqual(geometry_conv.VISUAL_GEOM_GROUP, mj_geom.group)


if __name__ == "__main__":
    unittest.main()
