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

from mjcf_to_sdformat.converters.geometry import (add_mjcf_visual_to_sdf,
    add_mjcf_collision_to_sdf)

from mjcf_to_sdformat.converters.material import add_mjcf_material_to_sdf

import sdformat as sdf

NUMBER_OF_SDF_LINK = 0
COLLISION_GEOM_GROUP = 3
VISUAL_GEOM_GROUP = 0

def add_mjcf_link_to_sdf(geom, inertial):
    global NUMBER_OF_SDF_LINK
    link = sdf.Link()
    if geom.name is not None:
        link.set_name(geom.name)
    else:
        link.set_name(geom.type + "_" + str(NUMBER_OF_SDF_LINK))
        NUMBER_OF_SDF_LINK = NUMBER_OF_SDF_LINK + 1


    if inertial is not None:
        inertial_pos = [0, 0, 0]
        inertial_euler = [0, 0, 0]
        if inertial.pos is not None:
            inertial_pos = inertial.pos
        if inertial.euler is not None:
            inertial_euler = inertial.euler
        inertial = Inertiald(MassMatrix3d(inertial.mass,
                                          Vector3d(inertial.fullinertia[0],
                                                   inertial.fullinertia[1],
                                                   inertial.fullinertia[2]),
                                          Vector3d(inertial.fullinertia[3],
                                                   inertial.fullinertia[4],
                                                   inertial.fullinertia[5])),
                             Pose3d(inertial_pos[0],
                                    inertial_pos[1],
                                    inertial_pos[2],
                                    inertial_euler[0],
                                    inertial_euler[1],
                                    inertial_euler[2]))
        link.set_inertial(inertial)

    if geom.group is None:
        visual = add_mjcf_visual_to_sdf(geom)
        material = add_mjcf_material_to_sdf(geom)
        if material is not None:
            visual.set_material(material)
        if visual is not None:
            link.add_visual(visual)

        col = add_mjcf_collision_to_sdf(geom)
        if col is not None:
            link.add_collision(col)
    elif geom.group == VISUAL_GEOM_GROUP:
        visual = add_mjcf_visual_to_sdf(geom)
        material = add_mjcf_material_to_sdf(geom)
        if material is not None:
            visual.set_material(material)
        if visual is not None:
            link.add_visual(visual)
    elif geom.group == COLLISION_GEOM_GROUP:
        col = add_mjcf_collision_to_sdf(geom)
        if col is not None:
            link.add_collision(col)

    return link
