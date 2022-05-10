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

from sdformat_mjcf.sdf_kinematics import KinematicHierarchy
from sdformat_mjcf.converters.link import add_link
from sdformat_mjcf.converters.joint import add_joint
from sdformat_mjcf.converters.light import add_light


def add_model(mjcf_out, model):
    kin_hierarchy = KinematicHierarchy(model)

    def convert_node(body, node):
        pose = model.raw_pose()
        child_body = add_link(body,
                              node.link,
                              node.parent_node.link.name(),
                              pose=pose)
        if child_body.geom[0].type != "plane":
            add_joint(child_body, node.joint)

        for cn in node.child_nodes:
            convert_node(child_body, cn)

    for cn in kin_hierarchy.world_node.child_nodes:
        convert_node(mjcf_out.worldbody, cn)

    return mjcf_out
