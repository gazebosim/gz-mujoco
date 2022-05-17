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

from ignition.math import Pose3d, Vector3d, Quaterniond

import math

from mjcf_to_sdformat.converters.joint import add_mjcf_joint_to_sdf
from mjcf_to_sdformat.converters.link import add_mjcf_link_to_sdf
from mjcf_to_sdformat.converters.light import add_mjcf_light_to_sdf

import sdformat_mjcf_utils.sdf_utils as su

import sdformat as sdf


NUMBER_OF_SDF_MODEL = 0

def add_link(body, model, main_body, parent_name):
    global NUMBER_OF_SDF_MODEL
    model_pose = [0, 0, 0]
    model_euler = [0, 0, 0]
    print(body)
    if body.joint is not None:
        for joint in body.joint:
            if main_body:
                joint = add_mjcf_joint_to_sdf(joint, None, body.geom[0].name)
            else:
                joint = add_mjcf_joint_to_sdf(joint, parent_name, body.geom[0].name)

            if joint is not None:
                model.add_joint(joint)
        if len(body.joint) != len(body.geom):
            for geom_number in range(1, len(body.geom)):
                print(geom_number)
                joint_sdf = sdf.Joint()
                joint_sdf.set_type(sdf.Joint.JointType.FIXED)
                joint_sdf.set_parent_link_name(body.geom[0].name)
                joint_sdf.set_child_link_name(body.geom[geom_number].name)
                joint_sdf.set_name("fixed_" + body.geom[0].name + "_" + body.geom[geom_number].name)
                model.add_joint(joint_sdf)

    if main_body:
        if body.pos is not None:
            model_pose = body.pos
        if body.euler is not None:
            model_euler = body.euler
        model.set_raw_pose(Pose3d(su.list_to_vec3d(model_pose),
                           Quaterniond(su.list_to_vec3d(model_euler))))

    for geom in body.geom:
        model.set_name("model_" + str(NUMBER_OF_SDF_MODEL))
        link = add_mjcf_link_to_sdf(geom, body.inertial)

        link_pose = [0, 0, 0]
        link_euler = [0, 0, 0]
        if geom.pos is not None:
            link_pose = geom.pos
        if geom.euler is not None:
            link_euler = geom.euler

        if geom.fromto is not None:
            v1 = Vector3d(geom.fromto[0], geom.fromto[1], geom.fromto[2])
            v2 = Vector3d(geom.fromto[3], geom.fromto[4], geom.fromto[5])
            # v_dir = v2 - v1
            link_pose = Vector3d((v2.x() + v1.x()) / 2,
                                 (v2.y() + v1.y()) / 2,
                                 (v2.z() + v1.z()) / 2)
            # print(v_dir)
            # print(1 / (math.sqrt(1) * math.sqrt(v_dir.x() * v_dir.x() + v_dir.y() * v_dir.y() + v_dir.z() * v_dir.z())))
            # link_euler =  math.acos( 1 / (math.sqrt(1) * math.sqrt(v_dir.x() * v_dir.x() + v_dir.y() * v_dir.y() + v_dir.z() * v_dir.z())))
        if not main_body:
            if body.pos is not None:
                link_pose = su.list_to_vec3d(body.pos) + link_pose

        link.set_raw_pose(Pose3d(su.list_to_vec3d(link_pose),
                          Quaterniond(su.list_to_vec3d(link_euler))))
        model.add_link(link)
        NUMBER_OF_SDF_MODEL = NUMBER_OF_SDF_MODEL + 1
    # link_world = sdf.Link()
    # link_world.set_name("world")
    # model.add_link(link_world)
    if body.body is not None:
        for b in body.body:
            add_link(b, model, False, body.name)

def add_mjcf_model_to_sdf(mjcf_model, world):
    # print(mjcf_model.default)
    # print(mjcf_model.default.joint.get_attributes())
    # print(mjcf_model.default.geom)
    # for k, v in mjcf_model.default.joint.get_attributes().items():
    #     print(k, v)
    global NUMBER_OF_SDF_MODEL
    for geom in mjcf_model.worldbody.geom:
        model = sdf.Model()
        model.set_name(geom.name)
        link = add_mjcf_link_to_sdf(geom, None)
        model.set_static(True)
        model.add_link(link)
        world.add_model(model)
        NUMBER_OF_SDF_MODEL = NUMBER_OF_SDF_MODEL + 1

    for light in mjcf_model.worldbody.light:
        light_sdf = add_mjcf_light_to_sdf(light)
        world.add_light(light_sdf)

    for body in mjcf_model.worldbody.body:
        model = sdf.Model()

        add_link(body, model, True, body.name)

        world.add_model(model)
