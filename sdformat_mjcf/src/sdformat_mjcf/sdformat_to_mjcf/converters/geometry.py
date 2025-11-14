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

import hashlib
import sdformat as sdf
import tempfile
import os
from urllib.parse import urlparse
from pathlib import Path

from sdformat_mjcf.sdformat_to_mjcf.mesh_io import (
    convert_mesh_to_obj_multimesh,
)
from sdformat_mjcf.sdformat_to_mjcf.converters.material import add_material
import sdformat_mjcf.utils.sdf_utils as su

COLLISION_GEOM_GROUP = 3
VISUAL_GEOM_GROUP = 0

def _get_asset_paths(env_vars: list[str]):
    output = []
    for env_var in env_vars:
        val = os.getenv(env_var)
        if val is not None:
            output.extend(val.split(os.pathsep))
    return output

# TODO(azeey): libsdformat has a utility function for resolving URIs, but
# it doesn't have a python binding yet. Once that becomes available, this
# function should be rewritten to leverage that.
def _resolve_uri(mesh_shape: sdf.Mesh):
    uri = urlparse(mesh_shape.uri())
    uri_file_path = Path(uri.netloc + uri.path)
    if (uri_file_path.suffix == ""):
        raise RuntimeError("Unable to find the mesh extension {}"
                           .format(uri))
    if uri.scheme in ['http', 'https']:
        raise RuntimeError("Fuel meshes are not yet supported")
    elif uri.scheme in ['model', 'package']:
        # Substitute paths from GZ_SIM_RESOURCE_PATH and SDF_PATH and search
        env_vars = ["SDF_PATH", "GZ_SIM_RESOURCE_PATH"]
        asset_paths = _get_asset_paths(env_vars)
        for dir in asset_paths:
            mesh_file_path = Path(dir) / uri_file_path
            if mesh_file_path.exists():
                return mesh_file_path
        # If we got here, then it means the file was not found
        raise RuntimeError(
            f"Could not find {uri_file_path} in paths defined "
            f"in the environment variables {env_vars}"
        )
    if uri_file_path.is_absolute():
        return uri_file_path
    else:
        parent_dir = Path(mesh_shape.file_path()).parent
        return parent_dir / uri_file_path


def _set_mesh_inertia(mjcf_mesh, is_visual):
    if is_visual:
        # Some visual meshes are too thin and can cause inertia
        # calculation in Mujoco to fail. Mark them as "shell" to prevent
        # Mujoco from failing in compilation. The visual geometry will
        # not be used for inertia computation anyway since we explicitly
        # convert inertia from sdformat to mjcf.
        # https://github.com/google-deepmind/mujoco/issues/2455
        mjcf_mesh.inertia = "shell"


def _is_unsupported_mesh_geo(sdf_geom):
    if not sdf_geom.mesh_shape():
        return False
    mesh_shape = sdf_geom.mesh_shape()
    uri = _resolve_uri(mesh_shape)
    _, extension = os.path.splitext(uri)
    # Mujoco supports .obj, .stl and .msh
    # Out of these, .obj is only supported if it has a single mesh in it.
    # So treat it as unsupported for this check so these files are sanitized
    # for Mujoco.
    return extension not in [".stl", ".msh"]

def _generate_mesh_name(mesh_file_path: Path):
    hash = hashlib.sha256(str(mesh_file_path.resolve().parent).encode('utf-8')).hexdigest()
    return f"{hash}_{mesh_file_path.name}"

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
        name=su.find_unique_name(body, "geom", su.sanitize_identifier_name(name)),
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
        if _is_unsupported_mesh_geo(sdf_geom):
            raise RuntimeError(
                f"Call `convert_and_add_mesh` for unsupported mesh geo {mesh_shape.uri()}")
        mesh_file_path = _resolve_uri(mesh_shape)
        mesh_name = _generate_mesh_name(mesh_file_path)
        geom.type = "mesh"
        asset_loaded = geom.root.asset.find('mesh', mesh_name)
        if asset_loaded is None:
            geom.mesh = geom.root.asset.add('mesh', name=mesh_name,
                                            file=str(mesh_file_path))
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
        name=su.find_unique_name(body, "geom", su.sanitize_identifier_name(name)),
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
    mesh_file_path = _resolve_uri(sdf_mesh)
    mesh_name = _generate_mesh_name(mesh_file_path)
    print("Converting", name, " gen: ", mesh_name, " uri:", mesh_file_path)
    mesh_loaded = body.root.asset.find('mesh', mesh_name)
    material_asset_name = "material_" + mesh_name
    material_loaded = body.root.asset.find('material', material_asset_name)
    if mesh_loaded:
        print("Mesh already loaded", mesh_name)
        geom = _add_mesh_geom_with_assets(body, name, pose, mesh_loaded,
                                          mjcf_material_asset=material_loaded)
        return [geom]

    # Try converting mesh to sanitized .obj. This could result in multiple
    # geos, one per mesh in the input file.
    # Pass a nominal path to `convert_mesh_to_obj_multimesh`. If multiple
    # sub-meshes are present, only the file name without extension from this
    # nominal path will be used as a prefix for the output files.
    with tempfile.TemporaryDirectory() as tmp_dir:
        output_filepath = (Path(tmp_dir) / mesh_name).with_suffix(".obj")
        result = convert_mesh_to_obj_multimesh(
            str(mesh_file_path), str(output_filepath)
        )
        geom_list = []
        for path_raw, info in result.obj_files.items():
            path = Path(path_raw)
            sub_mesh_name = path.stem
            sub_mesh_loaded = body.root.asset.find('mesh', sub_mesh_name)
            sub_mesh_material_asset_name = "material_" + sub_mesh_name
            material_loaded = body.root.asset.find('material',
                                                sub_mesh_material_asset_name)
            if sub_mesh_loaded:
                print("Mesh already loaded", sub_mesh_name)
                geom_list.append(
                    _add_mesh_geom_with_assets(body, name, pose, sub_mesh_loaded,
                                            mjcf_material_asset=material_loaded)
                )
                continue

            print("Adding submesh:", sub_mesh_name, " OBJ: ", path)
            # Add mesh and material assets
            mesh = body.root.asset.add('mesh', name=sub_mesh_name, file=path_raw)
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
