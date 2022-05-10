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

"""Module to convert SDFormat MAterials to MJCF"""

from ignition.math import Vector3d

from sdformat import Pbr, PbrWorkflow, Material  # noqa: F401

import sdformat_mjcf.sdf_utils as su

import os

TEXTURE_NUMBER = 0


def add_material(geom, material):
    """
    Converts an SDFormat material to an MJCF material.

    :param mjcf.Element geom: The MJCF geom to add the material
    :param sdf.Material material: The SDF material to convert
    :return: The newly created MJCF material.
    :rtype: mjcf.Element
    """
    pbr = material.pbr_material()
    sp_color = material.specular()
    em_color = material.emissive()
    specular = (sp_color.r() + sp_color.g() + sp_color.b()) / 3.0
    emissive = (em_color.r() + em_color.g() + em_color.b()) / 3.0
    asset = geom.root.asset
    r_mat = None
    if pbr is not None:
        workflow = pbr.workflow(PbrWorkflow.PbrWorkflowType.METAL)
        if workflow is not None:
            if workflow.albedo_map():
                file_without_extension, extension = os.path.splitext(
                    os.path.basename(workflow.albedo_map()))
                if not extension:
                    raise RuntimeError("Unable to find the extension {}"
                                       .format(workflow.albedo_map()))

                texture_loaded = asset.find('texture',
                                            file_without_extension)
                if texture_loaded is None:
                    texture = asset.add('texture',
                                        name=file_without_extension,
                                        type='2d',
                                        file=workflow.albedo_map(),
                                        gridsize='1 1')
                    r_mat = asset.add("material",
                                      name="material_" + file_without_extension,
                                      texture=texture,
                                      texrepeat="1 1",
                                      texuniform=True,
                                      specular=specular,
                                      emission=emissive)
                else:
                    r_mat = asset.find('material',
                                       'material_' + file_without_extension)
    else:
        global TEXTURE_NUMBER
        diffuse = material.diffuse()
        ambient = material.ambient()
        texture = asset.add('texture',
                            name="texture_" + str(TEXTURE_NUMBER),
                            # rgb1=su.vec3d_to_list(Vector3d(diffuse.r(),
                            #                                diffuse.g(),
                            #                                diffuse.b())),
                            # rgb2=su.vec3d_to_list(Vector3d(ambient.r(),
                            #                                ambient.g(),
                            #                                ambient.b())),
                            type='2d',
                            builtin="checker",
                            width=512,
                            height=512,
                            gridsize='1 1')
        TEXTURE_NUMBER = TEXTURE_NUMBER + 1
        r_mat = asset.add("material",
                          name="material_" + str(TEXTURE_NUMBER),
                          texture=texture,
                          texrepeat="1 1",
                          texuniform=True,
                          specular=specular,
                          emission=emissive,
                          rgba=[diffuse.r(),
                             diffuse.g(),
                             diffuse.b(),
                             diffuse.a()])
    geom.material = r_mat
    return r_mat
