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

from sdformat_mjcf.sdformat_to_mjcf.sdf_kinematics import (
    KinematicHierarchy,
    FreeJoint,
)
from sdformat_mjcf.sdformat_to_mjcf.converters.link import add_link
from sdformat_mjcf.sdformat_to_mjcf.converters.joint import add_joint
from sdformat_mjcf.utils.sdf_utils import graph_resolver
import sdformat as sdf


def add_model(mjcf_root, model):
    """
    Converts a model from SDFormat to MJCF and add it to the given
    MJCF root.

    :param mjcf.RootElement mjcf_root: The MJCF root element to which the model
    is added.
    :param sdformat.Model model: The SDFormat model to be converted.
    :return: The newly created MJCF body.
    :rtype: mjcf.Element
    """

    kin_hierarchy = KinematicHierarchy(model)
    model_pose = graph_resolver.resolve_pose(model.semantic_pose())

    def convert_node(body, node, link_pose=None):
        child_body = add_link(body, node.link, node.parent_node.link.name(),
                              link_pose)

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
        if isinstance(node.joint, FreeJoint):
            should_add_exclusions = False
        else:
            is_fixed_joint = node.joint.type() == sdf.JointType.FIXED
            is_body_world = body.tag == mjcf.constants.WORLDBODY
            should_add_exclusions = is_fixed_joint and is_body_world

        for cn in node.child_nodes:
            grand_child_body = convert_node(child_body, cn)
            if should_add_exclusions:
                body.root.contact.add(
                    "exclude",
                    name=f"{child_body.name}_{grand_child_body.name}",
                    body1=child_body.name,
                    body2=grand_child_body.name,
                )
        return child_body

    for cn in kin_hierarchy.world_node.child_nodes:
        # Adjust the poses of each of the nodes to account for the model
        link_pose = graph_resolver.resolve_pose(cn.link.semantic_pose())
        new_link_pose = model_pose * link_pose
        convert_node(mjcf_root.worldbody, cn, new_link_pose)

    return mjcf_root
