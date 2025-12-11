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
import sdformat_mjcf.utils.sdf_utils as su


class StaticFixedJoint(sdf.Joint):
    """Joint that represents fixed joints created to connect links in static
    models to the worldbody."""
    def __init__(self):
        super().__init__()
        self.set_type(sdf.JointType.FIXED)


class FreeJoint(sdf.Joint):
    """Joint that represents free joints that are not explicitly in the
    SDFormat file, but are needed in MJCF."""
    pass


class LinkNode:
    """
    A Node that represents links/bodies in KinematicHierarchy defined below.
    """
    def __init__(self, link, parent=None, joint=FreeJoint()):
        """
        Initialize the node with an SDFormat link, an optional parent node and
        an optional joint
        :param sdformat.Link link: The SDFormat link associated with this node.
        :param LinkNode parent: The parent link node.
        :param sdformat.Joint joint: The SDFormat joint whose <child> tag
        refers to the link assocated with this node.
        """
        self.link = link
        self.parent_node = parent
        self.joint = joint
        self.child_nodes = []

        # Pose computed for top-level child model links
        self.resolved_pose = None

        # Scoped name assigned for child model links
        self.scoped_name = None

    def __repr__(self):
        child_repr = " ".join(str(node) for node in self.child_nodes)
        link_name = self.scoped_name if self.scoped_name else self.link.name()
        return f"{link_name}->({child_repr})"

    def add_child(self, node, joint):
        """
        Add a child node and set the node's parent_node attribute to this node.
        :param LinkNode node: The child node to be added
        :param sdformat.Joint joint: The SDFormat joint that whose <child> tag
        refers to the link assocated with this node.
        """
        node.parent_node = self
        node.joint = joint
        self.child_nodes.append(node)

    def remove_child(self, node):
        """
        Remove a child node. This also clears the node's parent_node attribute.
        :param LinkNode node: The child node to be removed.
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

        # Map of scoped link name to link. If this is a child model kin
        # hierarchy, it will be merged into the parent kin hierarchy map.
        self.scoped_named_link_map = {}

        self.link_to_node_dict = {self.world_link: self.world_node}

        # Recursively create child model kin hierarchies and merge them up.
        # When merging, all children of the child model kin hierarchy world
        # node are initially listed under the current world node. If there is a
        # joint with a child link in a child model, it will be re-parented when
        # joints are processed below.
        for mi in range(model.model_count()):
            child_model = model.model_by_index(mi)
            child_kh = KinematicHierarchy(child_model)
            for cn in child_kh.world_node.child_nodes:
                if cn.resolved_pose:
                    cn_link_pose = cn.resolved_pose
                else:
                    cn_link_pose = su.graph_resolver.resolve_pose(
                        cn.link.semantic_pose())
                child_pose = su.graph_resolver.resolve_pose(
                    child_model.semantic_pose())
                cn.resolved_pose = child_pose * cn_link_pose
                # Removing the child node from child_kh.world_node is
                # unnecessary and dangerous since it affects the iterable.
                self.world_node.add_child(cn, cn.joint)

            # Merge `scoped_named_link_map` and update scoped name for
            # child model nodes. `scoped_named_link_map` is required to process
            # joints whose child link is in a child model.
            for scoped_name, link in child_kh.scoped_named_link_map.items():
                link_scoped_named = child_model.name() + '::' + scoped_name
                self.scoped_named_link_map[link_scoped_named] = link
                child_kh_link_node = child_kh.link_to_node_dict[link]
                child_kh_link_node.scoped_name = link_scoped_named

            # Merge child model link to node map
            self.link_to_node_dict.update(child_kh.link_to_node_dict)

        # Start with every link being a child of world link. Later on, we will
        # process joints to build the hierarchy.
        for li in range(model.link_count()):
            node = LinkNode(model.link_by_index(li), self.world_node)
            self.link_to_node_dict[node.link] = node
            joint = StaticFixedJoint() if model.static() else FreeJoint()
            self.world_node.add_child(node, joint)
            self.scoped_named_link_map[node.link.name()] = node.link

        for ji in range(model.joint_count()):
            joint = model.joint_by_index(ji)
            parent_link_name = su.graph_resolver.resolve_parent_link_name(
                joint)
            parent = model.link_by_name(parent_link_name)
            if parent_link_name == "world":
                parent = self.world_link

            child_link_name = su.graph_resolver.resolve_child_link_name(joint)
            child_link = self.scoped_named_link_map[child_link_name]
            child_node = self.link_to_node_dict[child_link]
            
            if parent_link_name != "world":
                parent_node = self.link_to_node_dict[parent]
                if parent_node.resolved_pose:
                    parent_pose = parent_node.resolved_pose
                else:
                    parent_pose = su.graph_resolver.resolve_pose(
                        parent.semantic_pose())
                if child_node.resolved_pose:
                    child_pose = child_node.resolved_pose
                else:
                    child_pose = su.graph_resolver.resolve_pose(
                        child_link.semantic_pose())
                child_node.resolved_pose = (
                    parent_pose.inverse() * child_pose
                )

            self.world_node.remove_child(child_node)
            self.link_to_node_dict[parent].add_child(child_node, joint)
