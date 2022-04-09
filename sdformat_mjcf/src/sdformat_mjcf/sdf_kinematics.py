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

import sdformat as sdf


class LinkNode:
    """
    A Node that represents links/bodies in KinematicHierarchy defined below.
    """
    def __init__(self, link, parent=None, joint=None):
        """
        Initialize the node with an SDFormat link, an optional parent node and
        an optional joint
        :param sdformat.Link link: The SDFormat link associated with this node.
        :param LinkNode parent: The parent link node.
        :param sdformat.Joint joint: The SDFormat joint that whose <child> tag
        refers to the link assocated with this node.
        """
        self.link = link
        self.parent_node = parent
        self.joint = joint
        self.child_nodes = []

    def __repr__(self):
        child_repr = " ".join(str(node) for node in self.child_nodes)
        return f"{self.link.name()}->({child_repr})"

    def add_child(self, node, joint):
        """
        Add a child node.
        :param sdformat.Link link: The SDFormat link associated with this node.
        :param LinkNode parent: The parent link node.
        :param sdformat.Joint joint: The SDFormat joint that whose <child> tag
        refers to the link assocated with this node.
        """
        node.parent_node = self
        node.joint = joint
        self.child_nodes.append(node)

    def remove_child(self, node):
        """
        Remove a child node.
        :param sdformat.Link link: The SDFormat link associated with this node.
        """
        node.parent_node = None
        self.child_nodes.remove(node)


class KinematicHierarchy:
    """
    Used to build a kinematic hierarchy (currently, only tree) of links from an
    SDFormat Model. The root of the hierarchy is the world link, which
    corresponds to MJCFs worldbody. Unconnected links found in the model are
    made children of the world link connected by a free joint.
    """
    def __init__(self, model):
        """
        Initialize a Kinematicierarchy from an SDFormat Model

        :param sdformat.Model model: The SDFormat model for building the
        hierarchy.
        """
        self.world_link = sdf.Link()
        self.world_link.set_name("world")
        self.world_node = LinkNode(self.world_link)

        link_to_node_dict = {self.world_link: self.world_node}

        # Start with every link being a child of world link. Later on, we will
        # process joints build the hierarchy.
        for li in range(model.link_count()):
            node = LinkNode(model.link_by_index(li), self.world_node)
            link_to_node_dict[node.link] = node
            self.world_node.add_child(node, None)

        for ji in range(model.joint_count()):
            joint = model.joint_by_index(ji)
            errors, parent_link_name = joint.resolve_parent_link()
            parent = model.link_by_name(parent_link_name)
            if parent_link_name == "world":
                parent = self.world_link

            errors, child_link_name = joint.resolve_child_link()
            # TODO (azeey) We assume that the child link is in the current
            # model, i.e., not nested. Remove this assumption when we support
            # nesting.
            child_node = link_to_node_dict[model.link_by_name(child_link_name)]

            self.world_node.remove_child(child_node)
            link_to_node_dict[parent].add_child(child_node, joint)