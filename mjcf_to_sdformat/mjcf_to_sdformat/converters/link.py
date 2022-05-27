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

from ignition.math import Inertiald, MassMatrix3d, Vector3d, Pose3d

from mjcf_to_sdformat.converters.geometry import (mjcf_visual_to_sdf,
                                                  mjcf_collision_to_sdf)
from mjcf_to_sdformat.converters.light import mjcf_light_to_sdf

import sdformat as sdf
import sdformat_mjcf_utils.sdf_utils as su

NUMBER_OF_SDF_LINK = 0
COLLISION_GEOM_GROUP = 3
VISUAL_GEOM_GROUP = 0


def mjcf_geom_to_sdf(body):
    """
    Converts an MJCF body to a SDFormat.

    :param mjcf.Element body: The MJCF body
    :param mjcf.Element inertial: Inertial of the body
    :return: The newly created SDFormat link.
    :rtype: sdf.Link
    """
    link = sdf.Link()

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

    for geom in body.geom:
        # If the group is not defined then visual and collision is added
        if geom.group is None:
            visual = mjcf_visual_to_sdf(geom)
            if visual is not None:
                visual.set_name(su.prefix_name_with_index(
                    "visual", geom.name, NUMBER_OF_VISUAL))
                visual.set_raw_pose(su.get_pose_from_mjcf(geom))
                link.add_visual(visual)

            col = mjcf_collision_to_sdf(geom)
            if col is not None:
                col.set_name(su.prefix_name_with_index(
                    "collision", geom.name, NUMBER_OF_COLLISION))
                col.set_raw_pose(su.get_pose_from_mjcf(geom))
                link.add_collision(col)
        elif geom.group == VISUAL_GEOM_GROUP:
            visual = mjcf_visual_to_sdf(geom)
            if visual is not None:
                visual.set_name(su.prefix_name_with_index(
                    "visual", geom.name, NUMBER_OF_VISUAL))
                visual.set_raw_pose(su.get_pose_from_mjcf(geom))
                link.add_visual(visual)
        elif geom.group == COLLISION_GEOM_GROUP:
            col = mjcf_collision_to_sdf(geom)
            if col is not None:
                col.set_name(su.prefix_name_with_index(
                    "collision", geom.name, NUMBER_OF_COLLISION))
                col.set_raw_pose(su.get_pose_from_mjcf(geom))
                link.add_collision(col)

    for light in body.light:
        light_sdf = mjcf_light_to_sdf(light)
        link.add_light(light_sdf)

    return link
