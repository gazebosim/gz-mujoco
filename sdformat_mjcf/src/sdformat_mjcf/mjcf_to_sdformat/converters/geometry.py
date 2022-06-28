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

"""Module to convert MJCF geoms to SDFormat Collision/Visual geometries"""

import os
from ignition.math import Vector2d, Vector3d

import sdformat as sdf
import sdformat_mjcf.utils.sdf_utils as su

VISUAL_NUMBER = 0
COLLISION_NUMBER = 0

# SDFormat doesn't have infinite plane sizes, so we use a large number instead.
INFINITE_PLANE_SIZE = 1e6
MESH_OUTPUT_DIR = "meshes"


def mjcf_geom_to_sdf(geom):
    """
    Converts an MJCF geom to a SDFormat geometry.

    :param mjcf.Element geom: The MJCF geom
    :return: The newly created SDFormat geometry.
    :rtype: sdf.Geometry
    """
    sdf_geometry = sdf.Geometry()
    is_default_type = geom.type is None and geom.mesh is None
    if geom.type == "box":
        box = sdf.Box()
        if geom.fromto is None:
            box.set_size(su.list_to_vec3d(geom.size) * 2)
        else:
            raise RuntimeError(
                "Encountered unsupported shape type attribute 'fromto'")
        sdf_geometry.set_box_shape(box)
        sdf_geometry.set_type(sdf.GeometryType.BOX)
    elif geom.type == "capsule":
        capsule = sdf.Capsule()
        capsule.set_radius(geom.size[0])
        if geom.fromto is None:
            capsule.set_length(geom.size[1] * 2)
        else:
            v1 = Vector3d(geom.fromto[0], geom.fromto[1], geom.fromto[2])
            v2 = Vector3d(geom.fromto[3], geom.fromto[4], geom.fromto[5])
            length = v1.distance(v2)
            capsule.set_length(length)
        sdf_geometry.set_capsule_shape(capsule)
        sdf_geometry.set_type(sdf.GeometryType.CAPSULE)
    elif geom.type == "cylinder":
        cylinder = sdf.Cylinder()
        cylinder.set_radius(geom.size[0])
        if geom.fromto is None:
            cylinder.set_length(geom.size[1] * 2)
        else:
            v1 = Vector3d(geom.fromto[0], geom.fromto[1], geom.fromto[2])
            v2 = Vector3d(geom.fromto[3], geom.fromto[4], geom.fromto[5])
            length = v1.distance(v2)
            cylinder.set_length(length)
        sdf_geometry.set_cylinder_shape(cylinder)
        sdf_geometry.set_type(sdf.GeometryType.CYLINDER)
    elif geom.type == "ellipsoid":
        ellipsoid = sdf.Ellipsoid()
        if geom.fromto is None:
            ellipsoid.set_radii(su.list_to_vec3d(geom.size))
        else:
            raise RuntimeError(
                "Encountered unsupported shape type attribute 'fromto'")
        sdf_geometry.set_ellipsoid_shape(ellipsoid)
        sdf_geometry.set_type(sdf.GeometryType.ELLIPSOID)
    elif geom.type == "sphere" or is_default_type:
        sphere = sdf.Sphere()
        sphere.set_radius(geom.size[0])
        sdf_geometry.set_sphere_shape(sphere)
        sdf_geometry.set_type(sdf.GeometryType.SPHERE)
    elif geom.type == "plane":
        plane = sdf.Plane()
        geom_size = []
        for sz in geom.size[:2]:
            geom_size.append(sz * 2 if sz > 0 else INFINITE_PLANE_SIZE)
        plane.set_size(Vector2d(*geom_size))
        sdf_geometry.set_plane_shape(plane)
        sdf_geometry.set_type(sdf.GeometryType.PLANE)
    elif geom.type == "mesh" or geom.mesh is not None:
        mj_mesh = geom.mesh
        mesh = sdf.Mesh()
        scale = su.get_value_or_default(mj_mesh.scale, [1, 1, 1])
        mesh.set_scale(su.list_to_vec3d(scale))
        mesh.set_uri(
            os.path.join(MESH_OUTPUT_DIR,
                         su.get_asset_filename_on_disk(mj_mesh)))
        sdf_geometry.set_mesh_shape(mesh)
        sdf_geometry.set_type(sdf.GeometryType.MESH)
    else:
        raise RuntimeError(
            f"Encountered unsupported shape type {geom.type}")
    return sdf_geometry


def mjcf_visual_to_sdf(geom):
    """
    Converts MJCF geom to a SDFormat visual
    MJCF geom should be part of group `VISUAL_GEOM_GROUP`.

    :param mjcf.Element geom: The MJCF geom.
    :return: The newly created SDFormat visual.
    :rtype: sdformat.Visual
    """
    visual = sdf.Visual()
    if geom.name is not None:
        visual.set_name("visual_" + geom.name)
    else:
        global VISUAL_NUMBER
        visual.set_name("unnamed_visual_" + str(VISUAL_NUMBER))
        VISUAL_NUMBER = VISUAL_NUMBER + 1
    sdf_geometry = mjcf_geom_to_sdf(geom)
    if sdf_geometry is not None:
        visual.set_geometry(sdf_geometry)
    else:
        return None
    return visual


def mjcf_collision_to_sdf(geom):
    """
    Converts MJCF geom to a SDFormat collision
    MJCF geom should be part of group `COLLISION_GEOM_GROUP`.

    :param mjcf.Element geom: The MJCF geom.
    :return: The newly created SDFormat collision.
    :rtype: sdformat.Collision
    """
    col = sdf.Collision()
    if geom.name is not None:
        col.set_name("collision_" + geom.name)
    else:
        global COLLISION_NUMBER
        col.set_name("unnamed_collision_" + str(COLLISION_NUMBER))
        COLLISION_NUMBER = COLLISION_NUMBER + 1
    sdf_geometry = mjcf_geom_to_sdf(geom)
    if sdf_geometry is not None:
        col.set_geometry(sdf_geometry)
    else:
        return None
    friction = su.get_value_or_default(geom.friction, [1, 0.005, 0.0001])
    col.surface().friction().ode().set_mu(friction[0])
    return col
