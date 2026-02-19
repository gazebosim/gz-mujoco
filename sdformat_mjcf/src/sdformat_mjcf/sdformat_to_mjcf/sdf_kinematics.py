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
#
from gz.math import Pose3d
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

        # Pose computed for all links relative to the model frame. This is stored
        # to avoid recomputation.
        self.resolved_pose_wrt_model = Pose3d()


        # Pose computed for all links. The value would be relative to the
        # parent of the link. If the link has no joints, it is considered to be
        # connected to the world link with a free joint.
        self.resolved_pose_wrt_parent = None

        # The scoped name of the link relative to the kinematic hierarchy root.
        self.scoped_name = None

    def __repr__(self):
        child_repr = " ".join(str(node) for node in self.child_nodes)
        link_name = self.scoped_name if self.scoped_name else self.link.name()
        return f"{link_name}->[{self.resolved_pose_wrt_parent}]->({child_repr})"

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
        self.world_node.scoped_name = "world"

        self.link_to_node_dict = {self.world_link: self.world_node}

        # Recursively create child model kin hierarchies and merge them up.
        # When merging, all children of the child model kin hierarchy world
        # node are initially listed under the current world node. If there is a
        # joint with a child link in a child model, it will be re-parented when
        # joints are processed below.
        for mi in range(model.model_count()):
            child_model = model.model_by_index(mi)
            child_kh = KinematicHierarchy(child_model)
            child_model_pose = su.graph_resolver.resolve_pose(
                child_model.semantic_pose())
            for cn in child_kh.world_node.child_nodes:
                cn.resolved_pose_wrt_model = child_model_pose * cn.resolved_pose_wrt_model
                # Removing the child node from child_kh.world_node is
                # unnecessary and dangerous since it affects the iterable.
                self.world_node.add_child(cn, cn.joint)
                cn.scoped_name = child_model.name() + '::' + cn.scoped_name

            # Merge child model link to node map
            self.link_to_node_dict.update(child_kh.link_to_node_dict)

        # Start with every link being a child of world link. Later on, we will
        # process joints to build the hierarchy.
        for li in range(model.link_count()):
            node = LinkNode(model.link_by_index(li), self.world_node)
            node.scoped_name = node.link.name()
            node.resolved_pose_wrt_model = su.graph_resolver.resolve_pose(
                node.link.semantic_pose()
            )
            self.link_to_node_dict[node.link] = node
            joint = StaticFixedJoint() if model.static() else FreeJoint()
            self.world_node.add_child(node, joint)

        for ji in range(model.joint_count()):
            joint = model.joint_by_index(ji)
            parent_link_name = su.graph_resolver.resolve_parent_link_name(
                joint)
            parent = model.link_by_name(parent_link_name)
            if parent_link_name == "world":
                parent = self.world_link

            child_link_name = su.graph_resolver.resolve_child_link_name(joint)
            child_node = self.link_to_node_dict[
                model.link_by_name(child_link_name)
            ]

            if parent_link_name != "world":
                parent_node = self.link_to_node_dict[parent]
                parent_pose_inv = parent_node.resolved_pose_wrt_model.inverse()
                child_node.resolved_pose_wrt_parent = (
                    parent_pose_inv * child_node.resolved_pose_wrt_model
                )

            self.world_node.remove_child(child_node)
            self.link_to_node_dict[parent].add_child(child_node, joint)

        # Update the resolved pose of the remaining nodes in world_node since they don't have a parent.
        for node in self.world_node.child_nodes:
            node.resolved_pose_wrt_parent = node.resolved_pose_wrt_model

