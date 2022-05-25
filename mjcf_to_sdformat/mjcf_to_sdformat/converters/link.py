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

import math

from mjcf_to_sdformat.converters.geometry import (mjcf_visual_to_sdf,
                                                  mjcf_collision_to_sdf)

import sdformat as sdf
import sdformat_mjcf_utils.sdf_utils as su

NUMBER_OF_SDF_LINK = 0
COLLISION_GEOM_GROUP = 3
VISUAL_GEOM_GROUP = 0


def mjcf_geom_to_sdf(body, body_parent_name=None):
    """
    Converts an MJCF body to a SDFormat.

    :param mjcf.Element body: The MJCF body
    :param mjcf.Element inertial: Inertial of the body
    :return: The newly created SDFormat link.
    :rtype: sdf.Link
    """
    link = sdf.Link()
    if body_parent_name is not None:
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
        link.set_name("link_" + str(NUMBER_OF_SDF_LINK))
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

    NUMBER_OF_VISUAL = 0
    NUMBER_OF_COLLISION = 0

    link.set_raw_pose(su.get_pose_from_mjcf(body))

    def get_orientation(geom):
        """
        Get orientation from a MJCF geom when it's defined with "fromto"
        """
        if geom.fromto is not None:
            v1 = Vector3d(geom.fromto[0], geom.fromto[1], geom.fromto[2])
            v2 = Vector3d(geom.fromto[3], geom.fromto[4], geom.fromto[5])
            vec = (v1 - v2).normalize()
            z = Vector3d(0, 0, 1)
            q_real = z.cross(vec)
            s = q_real.length()
            quat = Quaterniond()
            if s < 1E-10:
                quat.set_x(1)
                quat.set_y(0)
                quat.set_z(0)
            ang = math.atan2(s, vec.z())
            quat.set_w(math.cos(ang / 2.0))
            quat.set_x(q_real.x() * math.sin(ang / 2.0))
            quat.set_y(q_real.y() * math.sin(ang / 2.0))
            quat.set_z(q_real.z() * math.sin(ang / 2.0))
            return quat.normalized()
        return Quaterniond()

    def get_pose(geom):
        """
        Get the translattion from a MJCF geom when it's defined with "fromto"
        """
        if geom.fromto is not None:
            v1 = Vector3d(geom.fromto[0], geom.fromto[1], geom.fromto[2])
            v2 = Vector3d(geom.fromto[3], geom.fromto[4], geom.fromto[5])
            v_abs = (v2.abs() - v1.abs()) / 2.0
            return Vector3d(v_abs.x(), v_abs.y(), v_abs.z())
        return Vector3d()

    def set_visual(geom):
        visual = mjcf_visual_to_sdf(geom)
        if visual is not None:
            visual.set_name(su.prefix_name_with_index(
                "visual", geom.name, NUMBER_OF_VISUAL))
            pose_form_to = Pose3d(get_pose(geom), get_orientation(geom))
            pose = pose_form_to * su.get_pose_from_mjcf(geom)
            visual.set_raw_pose(pose)
            link.add_visual(visual)

    def set_collision(geom):
        col = mjcf_collision_to_sdf(geom)
        if col is not None:
            col.set_name(su.prefix_name_with_index(
                "collision", geom.name, NUMBER_OF_COLLISION))
            pose_form_to = Pose3d(get_pose(geom), get_orientation(geom))
            pose = pose_form_to * su.get_pose_from_mjcf(geom)
            col.set_raw_pose(pose)
            col.set_raw_pose(su.get_pose_from_mjcf(geom))
            link.add_collision(col)

    for geom in body.geom:
        # If the group is not defined then visual and collision is added
        if geom.group is None:
            set_visual(geom)
            set_collision(geom)
        elif geom.group == VISUAL_GEOM_GROUP:
            set_visual(geom)
        elif geom.group == COLLISION_GEOM_GROUP:
            set_collision(geom)
    return link
