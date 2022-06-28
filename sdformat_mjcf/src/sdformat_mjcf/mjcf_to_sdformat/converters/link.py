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

from ignition.math import (Inertiald, MassMatrix3d, Vector3d, Pose3d,
                           Quaterniond)

from sdformat_mjcf.mjcf_to_sdformat.converters.geometry import (
    mjcf_visual_to_sdf,
    mjcf_collision_to_sdf,
)
from sdformat_mjcf.mjcf_to_sdformat.converters.light import mjcf_light_to_sdf

from sdformat_mjcf.mjcf_to_sdformat.converters.material import (
    mjcf_material_to_sdf,
)

import sdformat as sdf
import sdformat_mjcf.utils.sdf_utils as su

NUMBER_OF_SDF_LINK = 0
COLLISION_GEOM_GROUP = 3
VISUAL_GEOM_GROUP = 0


def mjcf_body_to_sdf(body, physics, body_parent_name=None, modifiers=None):
    """
    Converts an MJCF body to a SDFormat.

    :param mjcf.Element body: The MJCF body
    :param mujoco.Physics physics: Mujoco Physics
    :param mjcf.Element inertial: Inertial of the body
    :param sdformat_mjcf.utils.MjcfModifiers modifiers: Modifiers that apply
    default classes to elements.
    :return: The newly created SDFormat link.
    :rtype: sdf.Link
    """
    link = sdf.Link()
    if body_parent_name is not None and body_parent_name != "world":
        link.set_pose_relative_to(body_parent_name)

    body_name = None
    try:
        body_name = body.name
    except AttributeError:
        pass

    # If name is not defined, we add a dummy name
    if body_name is not None:
        link.set_name(body_name)
    else:
        global NUMBER_OF_SDF_LINK
        link_name = "link_" + str(NUMBER_OF_SDF_LINK)
        link.set_name(link_name)
        NUMBER_OF_SDF_LINK = NUMBER_OF_SDF_LINK + 1

    inertial = None
    try:
        inertial = body.inertial
    except AttributeError:
        pass

    # If inertial is defined, then it added to the link
    if inertial is not None:
        inertial_pos = [0, 0, 0]
        inertial_euler = [0, 0, 0]
        fullinertia = [1, 1, 1, 0, 0, 0]

        if inertial.pos is not None:
            inertial_pos = inertial.pos
        inertial_euler = su.get_rotation(inertial)
        if inertial.fullinertia is not None:
            fullinertia = inertial.fullinertia
        if inertial.diaginertia is not None:
            fullinertia = [inertial.diaginertia[0],
                           inertial.diaginertia[1],
                           inertial.diaginertia[2],
                           0,
                           0,
                           0]

        # Mass is required, we don't need to check it
        inertial = Inertiald(MassMatrix3d(inertial.mass,
                                          Vector3d(fullinertia[0],
                                                   fullinertia[1],
                                                   fullinertia[2]),
                                          Vector3d(fullinertia[3],
                                                   fullinertia[4],
                                                   fullinertia[5])),
                             Pose3d(inertial_pos[0],
                                    inertial_pos[1],
                                    inertial_pos[2],
                                    inertial_euler.roll(),
                                    inertial_euler.pitch(),
                                    inertial_euler.yaw()))
        link.set_inertial(inertial)
    else:
        try:
            body_inertia = physics.named.model.body_inertia[body.name]
            inertia_pos = physics.named.model.body_ipos[body.name]
            inertia_quat = physics.named.model.body_iquat[body.name]
            inertial = Inertiald(
                MassMatrix3d(physics.named.model.body_mass[body.name],
                             Vector3d(body_inertia[0],
                                      body_inertia[1],
                                      body_inertia[2]),
                             Vector3d(0, 0, 0)),
                Pose3d(su.list_to_vec3d(inertia_pos),
                       su.wxyz_list_to_quat(inertia_quat)))
            link.set_inertial(inertial)
        except AttributeError:
            pass

    link.set_raw_pose(su.get_pose_from_mjcf(body))

    def get_orientation(geom):
        """
        Get orientation from a MJCF geom when it's defined with "fromto"

        :param mjcf.Element geom: MJCF geom to extract the orientation
        :return: The newly created quaterion.
        :rtype ignition.math.Quateriond
        """
        if geom.fromto is not None:
            v1 = Vector3d(geom.fromto[0], geom.fromto[1], geom.fromto[2])
            v2 = Vector3d(geom.fromto[3], geom.fromto[4], geom.fromto[5])
            vec = (v1 - v2).normalize()
            z = Vector3d(0, 0, 1)
            quat = Quaterniond()
            quat.set_from_2_axes(z, vec)
            return quat
        else:
            return su.get_pose_from_mjcf(geom).rot()

    def get_position(geom):
        """
        Get the translattion from a MJCF geom when it's defined with "fromto"

        :param mjcf.Element geom: MJCF geom to extract the position
        :return: The newly created Vector3d.
        :rtype ignition.math.Vector3d
        """
        if geom.fromto is not None:
            v1 = Vector3d(geom.fromto[0], geom.fromto[1], geom.fromto[2])
            v2 = Vector3d(geom.fromto[3], geom.fromto[4], geom.fromto[5])
            return (v1 + v2) / 2.0
        else:
            return su.get_pose_from_mjcf(geom).pos()

    def set_visual(geom, index):
        visual = mjcf_visual_to_sdf(geom)
        if visual is not None:
            visual.set_name(su.prefix_name_with_index(
                "visual", geom.name, index))
            material = mjcf_material_to_sdf(geom)
            if material is not None:
                visual.set_material(material)
            pose = Pose3d(get_position(geom), get_orientation(geom))
            visual.set_raw_pose(pose)
            link.add_visual(visual)

    def set_collision(geom, index):
        col = mjcf_collision_to_sdf(geom)
        if col is not None:
            col.set_name(su.prefix_name_with_index(
                "collision", geom.name, index))
            pose = Pose3d(get_position(geom), get_orientation(geom))
            col.set_raw_pose(pose)
            link.add_collision(col)

    visual_index = 0
    collision_index = 0
    for ind, geom in enumerate(body.geom):
        if modifiers is not None:
            modifiers.apply_modifiers_to_element(geom)
        # If the group is not defined then visual and collision is added
        if geom.group is None:
            set_visual(geom, visual_index)
            set_collision(geom, collision_index)
            visual_index += 1
            collision_index += 1
        elif geom.group == VISUAL_GEOM_GROUP:
            set_visual(geom, visual_index)
            visual_index += 1
        elif geom.group == COLLISION_GEOM_GROUP:
            set_collision(geom, collision_index)
            collision_index += 1

    for light in body.light:
        if modifiers is not None:
            modifiers.apply_modifiers_to_element(light)
        light_sdf = mjcf_light_to_sdf(light)
        link.add_light(light_sdf)
    return link
