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

from dm_control import mjcf

from sdformat_mjcf.sdf_kinematics import KinematicHierarchy
from sdformat_mjcf.converters.link import add_link
from sdformat_mjcf.converters.joint import add_joint, JointType


def add_model(mjcf_out, model):
    kin_hierarchy = KinematicHierarchy(model)
    mjcf_out.model = model.name()

    def convert_node(body, node):
        child_body = add_link(body, node.link, node.parent_node.link.name())
        add_joint(child_body, node.joint)

        # Geoms added to bodies attached to the worldbody without a
        # joint (a fixed joint in SDFormat) are treated as belonging to
        # worldbody. This means that the collision filtering rule that
        # applies to geoms in bodies connected by a joint may not apply to
        # these geoms. For example, let body A be a child of worldbody
        # without a joint and B be a child of A with a revolute joint. Even
        # though A and B are connected by a joint, by default, their geoms
        # will collide with eachother since the geoms of A are considered
        # to belong to worldbody. To avoid this problem, we create contact
        # exclusions between A and B.
        should_add_exclusions = (node.joint.type() == JointType.FIXED
                                 and body.tag == mjcf.constants.WORLDBODY)

        for cn in node.child_nodes:
            grand_child_body = convert_node(child_body, cn)
            if should_add_exclusions:
                body.root.contact.add(
                    "exclude",
                    name=f"{child_body.name}_{grand_child_body.name}",
                    body1=child_body.name,
                    body2=grand_child_body.name)
        return child_body

    for cn in kin_hierarchy.world_node.child_nodes:
        convert_node(mjcf_out.worldbody, cn)

    return mjcf_out
