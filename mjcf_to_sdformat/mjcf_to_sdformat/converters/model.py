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

from ignition.math import Pose3d, Quaterniond, Vector3d

from mjcf_to_sdformat.converters.link import add_mjcf_geom_to_sdf
import sdformat_mjcf_utils.sdf_utils as su

import sdformat as sdf

NUMBER_OF_SDF_MODEL = 0

def add_link(body, model, main_body, parent_name):
    """
    Get a body and iterate all the geoms to convert them in SDFormat links

    :param mjcf.Element body: The MJCF body
    :param sdf.Model model: Model to add the links
    :param boolean main_body: True if the geom is a direct child of worldbody
        tag
    :param string parent_name: Parent name, this is usefull to define joints
    """
    model_pose = [0, 0, 0]
    model_euler = [0, 0, 0]

    if main_body:
        if body.pos is not None:
            model_pose = body.pos
        if body.euler is not None:
            model_euler = body.euler
        model.set_raw_pose(Pose3d(su.list_to_vec3d(model_pose),
                           Quaterniond(su.list_to_vec3d(model_euler))))

    for geom in body.geom:
        global NUMBER_OF_SDF_MODEL
        if body.name is not None:
            model.set_name(body.name)
        else:
            model.set_name("model_" + str(NUMBER_OF_SDF_MODEL))
            NUMBER_OF_SDF_MODEL = NUMBER_OF_SDF_MODEL + 1
        link = add_mjcf_geom_to_sdf(geom, body.inertial)

        link_pose = [0, 0, 0]
        link_euler = [0, 0, 0]
        if geom.pos is not None:
            link_pose = geom.pos
        if geom.euler is not None:
            link_euler = geom.euler

        if geom.fromto is not None:
            # TODO(ahcorde): Add Orientation
            v1 = Vector3d(geom.fromto[0], geom.fromto[1], geom.fromto[2])
            v2 = Vector3d(geom.fromto[3], geom.fromto[4], geom.fromto[5])
            link_pose = Vector3d((v2.x() + v1.x()) / 2,
                                 (v2.y() + v1.y()) / 2,
                                 (v2.z() + v1.z()) / 2)

        if not main_body:
            if body.pos is not None:
                link_pose = su.list_to_vec3d(body.pos) + link_pose

        link.set_raw_pose(Pose3d(su.list_to_vec3d(link_pose),
                          Quaterniond(su.list_to_vec3d(link_euler))))
        model.add_link(link)

    # Get subbodies
    if body.body is not None:
        for b in body.body:
            add_link(b, model, False, body.name)


def add_mjcf_model_to_sdf(mjcf_model, world):
    """
    Convert a MJCF model to a SDFormat world

    :param mjcf.Element mjcf_model: The MJCF model
    :param sdf.World world: SDF World to add the models
    """
    for geom in mjcf_model.worldbody.geom:
        model = sdf.Model()
        if geom.name is not None:
            model.set_name(geom.name)
        else:
            global NUMBER_OF_SDF_MODEL
            model.set_name("model_" + str(NUMBER_OF_SDF_MODEL))
            NUMBER_OF_SDF_MODEL = NUMBER_OF_SDF_MODEL + 1

        link = add_mjcf_geom_to_sdf(geom, None)
        model.set_static(True)
        model.add_link(link)
        world.add_model(model)

    for body in mjcf_model.worldbody.body:
        model = sdf.Model()

        add_link(body, model, True, body.name)

        world.add_model(model)
