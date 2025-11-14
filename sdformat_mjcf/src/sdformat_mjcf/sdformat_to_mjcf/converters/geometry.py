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

from sdformat_mjcf.sdformat_to_mjcf.mesh_io import (
    convert_mesh_to_obj_multimesh,
)
from sdformat_mjcf.sdformat_to_mjcf.converters.material import add_material
import sdformat_mjcf.utils.sdf_utils as su

COLLISION_GEOM_GROUP = 3
VISUAL_GEOM_GROUP = 0


def _set_mesh_inertia(mjcf_mesh, is_visual):
    if is_visual:
        # Some visual meshes are too thin and can cause inertia
        # calculation in Mujoco to fail. Mark them as "shell" to prevent
        # Mujoco from failing in compilation. The visual geometry will
        # not be used for inertia computation anyway since we explicitly
        # convert inertia from sdformat to mjcf.
        # https://github.com/google-deepmind/mujoco/issues/2455
        mjcf_mesh.inertia = "shell"


def _validate_uri(uri):
    if 'http://' in uri or 'https://' in uri:
        raise RuntimeError("Fuel meshes are not yet supported")
    if 'model://' in uri:
        prefix = 'model://'
        # TODO: Support sdf::ParserConfig::AddURIPath to resolve URIs.
        return uri.replace(prefix, '', 1)
    return uri


def _is_unsupported_mesh_geo(sdf_geom):
    if not sdf_geom.mesh_shape():
        return False
    mesh_shape = sdf_geom.mesh_shape()
    uri = _validate_uri(mesh_shape.uri())
    _, basename = os.path.split(uri)
    filename, extension = os.path.splitext(basename)
    # Mujoco supports .obj, .stl and .msh
    # Out of these, .obj is only supported if it has a single mesh in it.
    # So treat it as unsupported for this check so these files are sanitized
    # for Mujoco.
    return extension not in [".stl", ".msh"]


def add_geometry(body, name, pose, sdf_geom, is_visual=False):
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
        uri = _validate_uri(mesh_shape.uri())
        if _is_unsupported_mesh_geo(sdf_geom):
            raise RuntimeError(
                f"Call `convert_and_add_mesh` for unsupported mesh geo {uri}")
        extension_tokens = os.path.basename(uri).split(".")
        if (len(extension_tokens) == 1):
            raise RuntimeError("Unable to find the mesh extension {}"
                               .format(uri))
        file_without_extension = os.path.splitext(os.path.basename(uri))[0]
        geom.type = "mesh"
        asset_loaded = geom.root.asset.find('mesh', file_without_extension)
        if asset_loaded is None:
            dirname = os.path.dirname(mesh_shape.file_path())
            mesh_file_path = os.path.join(dirname, uri)
            geom.mesh = geom.root.asset.add('mesh',
                                            file=mesh_file_path)
            _set_mesh_inertia(geom.mesh, is_visual)
        else:
            geom.mesh = asset_loaded
        geom.mesh.scale = su.vec3d_to_list(mesh_shape.scale())
    else:
        raise RuntimeError(
            f"Encountered unsupported shape type {sdf_geom.type()}")

    return geom


def _add_mesh_geom_with_assets(body, name, pose, mjcf_mesh_asset,
                               mjcf_material_asset=None):
    geom = body.add(
        "geom",
        name=su.find_unique_name(body, "geom", name),
        pos=su.vec3d_to_list(pose.pos()),
        euler=su.quat_to_euler_list(pose.rot()),
    )
    geom.type = "mesh"
    geom.mesh = mjcf_mesh_asset
    if mjcf_material_asset:
        geom.material = mjcf_material_asset
    return geom


def convert_and_add_mesh(body, name, pose, sdf_mesh, is_visual=False):
    # Check if asset was loaded already with the uri key. If so, just add the
    # asset. This can happen if the converted mesh has a single sub-mesh,
    # which was loaded already.
    uri = _validate_uri(sdf_mesh.uri())
    file_without_extension = os.path.splitext(os.path.basename(uri))[0]
    mesh_asset_name = file_without_extension
    mesh_loaded = body.root.asset.find('mesh', mesh_asset_name)
    material_asset_name = "material_" + file_without_extension
    material_loaded = body.root.asset.find('material', material_asset_name)
    if mesh_loaded:
        geom = _add_mesh_geom_with_assets(body, name, pose, mesh_loaded,
                                          mjcf_material_asset=material_loaded)
        return [geom]

    # Try converting mesh to sanitized .obj. This could result in multiple
    # geos, one per mesh in the input file.
    dirname = os.path.dirname(sdf_mesh.file_path())
    mesh_file_path = os.path.join(dirname, uri)
    # Pass a nominal path to `convert_mesh_to_obj_multimesh`. If multiple
    # sub-meshes are present, only the file name without extension from this
    # nominal path will be used as a prefix for the output files.
    converted_path = os.path.join(dirname, file_without_extension + ".obj")
    print(f"Converting {mesh_file_path} to {converted_path}")
    result = convert_mesh_to_obj_multimesh(mesh_file_path, converted_path)
    geom_list = []
    for path, info in result.obj_files.items():
        output_file_without_extension = os.path.splitext(
            os.path.basename(path))[0]
        sub_mesh_loaded = body.root.asset.find('mesh',
                                               output_file_without_extension)
        sub_mesh_material_asset_name = (
            "material_" + output_file_without_extension)
        material_loaded = body.root.asset.find('material',
                                               sub_mesh_material_asset_name)
        if sub_mesh_loaded:
            geom_list.append(
                _add_mesh_geom_with_assets(body, name, pose, sub_mesh_loaded,
                                           mjcf_material_asset=material_loaded)
            )
            continue

        # Add mesh and material assets
        mesh = body.root.asset.add('mesh', file=path)
        _set_mesh_inertia(mesh, is_visual)
        mesh.scale = su.vec3d_to_list(sdf_mesh.scale())
        material_asset = None
        if is_visual:
            material_asset = body.root.asset.add(
                "material", name=sub_mesh_material_asset_name,
                specular=info.mat.specular, shininess=info.mat.shininess,
                rgba=info.mat.rgba)
        geom_list.append(
            _add_mesh_geom_with_assets(body, name, pose, mesh,
                                       mjcf_material_asset=material_asset))

    return geom_list


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
    if _is_unsupported_mesh_geo(col.geometry()):
        sdf_mesh = col.geometry().mesh_shape()
        geoms = convert_and_add_mesh(body, col.name(), pose, sdf_mesh,
                                     is_visual=False)
    else:
        geom = add_geometry(body, col.name(), pose, col.geometry(),
                            is_visual=False)
        geoms = [geom]
    for geom in geoms:
        geom.group = COLLISION_GEOM_GROUP
        apply_surface_to_geometry(geom, col.surface())
    if len(geoms) == 1:
        return geoms[0]
    return geoms


def add_visual(body, vis):
    """
    Converts an SDFormat visual to an MJCF geom and add it to the given
    body. To differentiate Visual geoms from Collision geoms, we assign
    Visual geoms group `VISUAL_GEOM_GROUP`.

    :param mjcf.Element body: The MJCF body to which the geom is added.
    :param sdformat.Visual vis: Visual object to be converted.
    :return: The newly created MJCF geom.
    :rtype: mjcf.Element or list of mjcf
    """
    sem_pose = vis.semantic_pose()
    pose = su.graph_resolver.resolve_pose(sem_pose)
    if _is_unsupported_mesh_geo(vis.geometry()):
        sdf_mesh = vis.geometry().mesh_shape()
        geoms = convert_and_add_mesh(body, vis.name(), pose, sdf_mesh,
                                     is_visual=True)
    else:
        geom = add_geometry(body, vis.name(), pose, vis.geometry(),
                            is_visual=True)
        geoms = [geom]

    for geom in geoms:
        geom.group = VISUAL_GEOM_GROUP
        # Visual geoms do not collide with any other geom, so we set their
        # contype and conaffinity to 0.
        geom.contype = 0
        geom.conaffinity = 0
    if vis.material() is not None:
        mjcf_mat = add_material(geom.root, vis.material())
        for geom in geoms:
            geom.material = mjcf_mat
    if len(geoms) == 1:
        return geoms[0]
    return geoms
