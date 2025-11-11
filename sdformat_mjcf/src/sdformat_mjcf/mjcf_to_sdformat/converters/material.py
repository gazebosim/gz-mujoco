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

import logging
import os

import sdformat as sdf

import sdformat_mjcf.utils.sdf_utils as su

TEXTURE_OUTPUT_DIR = "materials/textures"


def mjcf_material_to_sdf(geom):
    """
    Convert a MJCF material to a SDFormat material

    :param mjcf.Element geom: The MJCF geom to extract the material
    :return: The newly created SDFormat material.
    :rtype: sdf.Material
    """
    material = sdf.Material()
    if geom.material is not None:
        material_rgba = geom.material.rgba
        if material_rgba is None:
            material_rgba = [1] * 4

        material.set_diffuse(su.rgba_to_color(material_rgba))
        material.set_ambient(su.rgba_to_color(material_rgba))
        if geom.material.specular is not None:
            material.set_specular(
                su.rgba_to_color([geom.material.specular] * 3 + [1]))
        else:
            material.set_specular(su.rgba_to_color([0.5] * 3 + [1]))
        if geom.material.emission is not None:
            emission = [geom.material.emission * color for color in
                        material_rgba[:3]]
            material.set_emissive(su.rgba_to_color(emission + [1]))
        else:
            material.set_emissive(su.rgba_to_color([0] * 3 + [1]))

        if geom.material.texture is not None:
            if geom.material.texture.gridsize is not None:
                if (geom.material.texture.gridsize == [1, 1]).all():
                    logging.warning("Texture gridsize is not [1 1]. This"
                                    "might generate wrong results.")

            if geom.material.texture.gridlayout is not None:
                logging.warning("Texture gridlayout is set. This feature is "
                                "not supported.")

            if geom.material.texture.builtin is not None:
                logging.warning("Texture builtin is set, generation of "
                                "procedural textures is not supported.")

            if geom.material.texture.file is not None:
                pbr = sdf.Pbr()
                workflow = sdf.PbrWorkflow()
                workflow.set_type(sdf.PbrWorkflowType.METAL)
                filename = su.get_asset_filename_on_disk(geom.material.texture)
                workflow.set_albedo_map(
                    os.path.join(TEXTURE_OUTPUT_DIR, filename))
                pbr.set_workflow(workflow.type(), workflow)
                material.set_pbr_material(pbr)

    if geom.rgba is not None:
        material.set_diffuse(su.rgba_to_color(geom.rgba))
        material.set_ambient(su.rgba_to_color(geom.rgba))
        material.set_specular(su.rgba_to_color(geom.rgba))
        return material

    if geom.material is None and geom.rgba is None:
        material.set_diffuse(su.rgba_to_color([0.5, 0.5, 0.5, 1]))
        material.set_ambient(su.rgba_to_color([0.5, 0.5, 0.5, 1]))
        material.set_specular(su.rgba_to_color([0.5, 0.5, 0.5, 1]))

    return material
