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

"""Module to convert SDFormat Collision/Visual geometries to MJCF geoms"""

import os

import sdformat_mjcf.utils.sdf_utils as su

COLLISION_GEOM_GROUP = 3
VISUAL_GEOM_GROUP = 0


def add_geometry(body, name, pose, sdf_geom):
    """
    Converts an SDFormat geometry to an MJCF geom and add it to the given body.

    :param mjcf.Element body: The MJCF body to which the geom is added.
    :param str name: Name of the geom (obtained from the name of the SDFormat
            Collision or Visual).
    :param sdformat.Pose3d pose: Resolved pose of the geom (obtained from the
            pose of the SDFormat Collision or Visual).
    :param sdformat.Geometry sdf_geom: Geometry object to be converted.
    :return: The newly created MJCF geom.
    :rtype: mjcf.Element
    """

    if sdf_geom is None:
        return
    geom = body.add(
        "geom",
        name=su.find_unique_name(body, "geom", name),
        pos=su.vec3d_to_list(pose.pos()),
        euler=su.quat_to_euler_list(pose.rot()),
    )

    if sdf_geom.box_shape():
        box_shape = sdf_geom.box_shape()
        geom.type = "box"
        geom.size = su.vec3d_to_list(box_shape.size() / 2.0)
    elif sdf_geom.capsule_shape():
        capsule_shape = sdf_geom.capsule_shape()
        geom.type = "capsule"
        geom.size = [capsule_shape.radius(), capsule_shape.length() / 2.0]
    elif sdf_geom.cylinder_shape():
        cylinder_shape = sdf_geom.cylinder_shape()
        geom.type = "cylinder"
        geom.size = [cylinder_shape.radius(), cylinder_shape.length() / 2.0]
    elif sdf_geom.ellipsoid_shape():
        ellipsoid_shape = sdf_geom.ellipsoid_shape()
        geom.type = "ellipsoid"
        geom.size = su.vec3d_to_list(ellipsoid_shape.radii())
    elif sdf_geom.plane_shape():
        plane_shape = sdf_geom.plane_shape()
        geom.type = "plane"
        # The third element of size defines the spacing between square grid
        # lines for rendering.
        # TODO (azeey) Consider making this configurable
        geom.size = su.vec2d_to_list(plane_shape.size() / 2.0) + [1]
    elif sdf_geom.sphere_shape():
        sphere_shape = sdf_geom.sphere_shape()
        geom.type = "sphere"
        geom.size = [sphere_shape.radius()]
    elif sdf_geom.mesh_shape():
        mesh_shape = sdf_geom.mesh_shape()
        uri = mesh_shape.uri()
        extension_tokens = os.path.basename(mesh_shape.uri()).split(".")
        if (len(extension_tokens) == 1):
            raise RuntimeError("Unable to find the mesh extension {}"
                               .format(uri))
        file_without_extension = os.path.splitext(
            os.path.basename(mesh_shape.uri()))[0]
        if 'http://' in uri or 'https://' in uri:
            raise RuntimeError("Fuel meshes are not yet supported")
        geom.type = "mesh"
        asset_loaded = geom.root.asset.find('mesh', file_without_extension)
        dirname = os.path.dirname(mesh_shape.file_path())
        mesh_file_path = os.path.join(dirname, uri)
        if asset_loaded is None:
            geom.mesh = geom.root.asset.add('mesh',
                                            file=mesh_file_path)
        else:
            geom.mesh = asset_loaded
    else:
        raise RuntimeError(
            f"Encountered unsupported shape type {sdf_geom.type()}")

    return geom


def apply_surface_to_geometry(geom, sdf_surface):
    """
    Applies surface parameters from an SDFormat surface to an MJCF geom.

    :param mjcf.Element geom: The MJCF geom to which the surface parameters
            are applied.
    :param sdformat.Surface sdf_surface: Surface object to be applied.
    """

    sdf_friction_ode = sdf_surface.friction().ode()
    # Use //surface/friction/ode/mu for sliding friction
    geom.friction = [sdf_friction_ode.mu(), 0.005, 0.0001]


def add_collision(body, col):
    """
    Converts an SDFormat collision to an MJCF geom and add it to the given
    body. To differentiate Collision geoms from Visual geoms, we assign
    Collision geoms group `COLLISION_GEOM_GROUP`.

    :param mjcf.Element body: The MJCF body to which the geom is added.
    :param sdformat.Collision col: Collision object to be converted.
    :return: The newly created MJCF geom.
    :rtype: mjcf.Element
    """
    sem_pose = col.semantic_pose()
    pose = su.graph_resolver.resolve_pose(sem_pose)
    geom = add_geometry(body, col.name(), pose, col.geometry())
    geom.group = COLLISION_GEOM_GROUP
    apply_surface_to_geometry(geom, col.surface())
    return geom


def add_visual(body, vis):
    """
    Converts an SDFormat visual to an MJCF geom and add it to the given
    body. To differentiate Visual geoms from Collision geoms, we assign
    Visual geoms group `VISUAL_GEOM_GROUP`.

    :param mjcf.Element body: The MJCF body to which the geom is added.
    :param sdformat.Visual vis: Visual object to be converted.
    :return: The newly created MJCF geom.
    :rtype: mjcf.Element
    """
    sem_pose = vis.semantic_pose()
    pose = su.graph_resolver.resolve_pose(sem_pose)
    geom = add_geometry(body, vis.name(), pose, vis.geometry())
    geom.group = VISUAL_GEOM_GROUP
    # Visual geoms do not collide with any other geom, so we set their contype
    # and conaffinity to 0.
    geom.contype = 0
    geom.conaffinity = 0
    return geom
