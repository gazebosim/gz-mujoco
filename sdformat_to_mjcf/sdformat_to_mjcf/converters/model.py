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

from sdformat_to_mjcf.sdf_kinematics import KinematicHierarchy
from sdformat_to_mjcf.converters.link import add_link
from sdformat_to_mjcf.converters.joint import add_joint
from sdformat_mjcf_utils.sdf_utils import graph_resolver


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
    mjcf_root.model = model.name()
    model_pose = graph_resolver.resolve_pose(model.semantic_pose())

    def convert_node(body, node):
        child_body = add_link(body,
                              node.link,
                              node.parent_node.link.name())

        add_joint(child_body, node.joint)

        for cn in node.child_nodes:
            convert_node(child_body, cn)

    for cn in kin_hierarchy.world_node.child_nodes:
        # Adjust the poses of each of the nodes to account for the model
        link_pose = graph_resolver.resolve_pose(cn.link.semantic_pose())
        new_link_pose = model_pose * link_pose
        cn.link.set_raw_pose(new_link_pose)
        cn.link.set_pose_relative_to("")
        convert_node(mjcf_root.worldbody, cn)

    return mjcf_root
