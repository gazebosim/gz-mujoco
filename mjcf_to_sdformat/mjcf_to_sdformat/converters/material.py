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

from ignition.math import Color

import sdformat as sdf

import sdformat_mjcf_utils.sdf_utils as su


def mjcf_material_to_sdf(geom):
    """
    Convert a MJCF material to a SDFormat material

    :param mjcf.Element geom: The MJCF geom to extract the material
    :return: The newly created SDFormat material.
    :rtype: sdf.Material
    """
    material = sdf.Material()
    if geom.material is not None:
        if geom.material.rgba is not None:
            material.set_diffuse(su.rgba_to_color(geom.material.rgba))
            material.set_ambient(su.rgba_to_color(geom.material.rgba))
            material.set_specular(su.rgba_to_color(geom.material.rgba))
            material.set_emissive(su.rgba_to_color(geom.material.rgba))
            return material
        elif geom.material.texture is not None:
            if geom.material.texture.file is not None:
                material = sdf.Material()
                material.set_diffuse(Color(1, 1, 1, 1))
                material.set_specular(Color(1, 1, 1, 1))
                pbr = sdf.Pbr()
                workflow = sdf.PbrWorkflow()
                workflow.set_type(sdf.PbrWorkflow.PbrWorkflowType.METAL)
                print(geom.material.texture.file.get_vfs_filename())
                workflow.set_albedo_map(
                    geom.material.texture.file.get_vfs_filename())
                pbr.set_workflow(workflow.type(), workflow)
                material.set_pbr_material(pbr)
                return material
    if geom.rgba is not None:
        material.set_diffuse(su.rgba_to_color(geom.rgba))
        material.set_ambient(su.rgba_to_color(geom.rgba))
        material.set_specular(su.rgba_to_color(geom.rgba))
        material.set_emissive(su.rgba_to_color(geom.rgba))
        return material
    return None
