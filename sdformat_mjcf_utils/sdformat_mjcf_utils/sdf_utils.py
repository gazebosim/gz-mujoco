# Copyright (C) 2022 Open Source Robotics Foundation

# Licensed under the Apache License, version 2.0 (the "License")
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#       http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Utility functions that aid in converting SDFormat and Gazebo Math types to
Python"""

import math
from ignition.math import Pose3d, Vector3d

NAME_DELIMITER = '_'


def vec3d_to_list(vec):
    """Convert a Vector3d object to a list.
    :param ignition.math.Vector3d vec: Vector3d object to be converted
    :return: List of values of the x, y, and z components of `vec`
    respectively.
    :rtype: list[float]
    """
    return [vec.x(), vec.y(), vec.z()]


def list_to_vec3d(list):
    """Convert a list to a Vector3d object.
    :param List of values of the x, y, and z components of `vec`
    respectively.
    :return: Vector3d object
    :rtype: ignition.math.Vector3d
    """
    return Vector3d(list[0], list[1], list[2])


def vec2d_to_list(vec):
    """
    Convert a Vector2d object to a list.
    :param ignition.math.Vector2d vec: Vector2d object to be converted.
    :return: List of values of the x, and y components of `vec` respectively.
    :rtype: list[float]
    """
    return [vec.x(), vec.y()]


def quat_to_list(quat):
    """
    Convert a Quaterniond object to a list in the order expected by MJCF.
    :param ignition.math.Quaterniond quat: Quaterniond object to be converted.
    :return: List of values of the quaternion in wxyz order.
    :rtype: list[float]
    """
    return [quat.w(), quat.x(), quat.y(), quat.z()]


def quat_to_euler_list(quat):
    """
    Convert a Quaterniond object to a list of Euler angles in degrees.
    :param ignition.math.Quaterniond quat: Quaterniond object to be converted.
    :return: List of values of the euler angles.
    :rtype: list[float]
    """
    return [math.degrees(val) for val in vec3d_to_list(quat.euler())]


def prefix_name(prefix, name):
    """
    Prefixes a given `name` with `prefix`.
    :param str prefix: The prefix. If this is None or "world", the name is
    returned without being prefixed.
    :param str name: Name to prefix.
    :return: Prefixed name.
    :rtype: str
    """
    if prefix is None or prefix == "world":
        return name
    return prefix + NAME_DELIMITER + name


class GraphResolverImplBase:
    """Interface for graph resolver implementors"""

    def resolve_pose(self, sem_pose, relative_to=None):
        raise NotImplementedError

    def resolve_axis_xyz(self, joint_axis):
        raise NotImplementedError

    def resolve_parent_link_name(self, joint):
        raise NotImplementedError

    def resolve_child_link_name(self, joint):
        raise NotImplementedError


class GraphResolverImpl(GraphResolverImplBase):
    """Implementation of different pose and frame resolution functions."""

    def resolve_pose(self, sem_pose, relative_to=None):
        """
        Resolves SDFormat poses from a SemanticPose object.
        :param sdformat.SemanticPose sem_pose: The SemanticPose object to be
        resolved.
        :param str relative_to: (Optional) The frame relative to which the pose
        is resolved.
        :return: The resolved pose.
        :rtype: ignition.math.Pose3d
        :raises RuntimeError: if an error is encountered when resolving the
        pose.
        """
        pose = Pose3d()
        if relative_to is None:
            self._handle_errors(sem_pose.resolve(pose))
        else:
            self._handle_errors(sem_pose.resolve(pose, relative_to))
        return pose

    def resolve_axis_xyz(self, joint_axis):
        """
        Resolves the xyz unit vector of SDFormat Joint axes.
        :param sdformat.JointAxis joint_axis: The JointAxis object to be
        resolved.
        :return: The resolved xyz vector.
        :rtype: ignition.math.Vector3d
        :raises RuntimeError: if an error is encountered when resolving the
        vector.
        """
        xyz_vec = Vector3d()
        self._handle_errors(joint_axis.resolve_xyz(xyz_vec))
        return xyz_vec

    def resolve_parent_link_name(self, joint):
        """
        Resolves the parent link name of an SDFormat Joint.
        :param sdformat.Joint joint: The Joint object.
        :return: The resolved name of the parent link.
        :rtype: str
        :raises RuntimeError: if an error is encountered when resolving the
        the link.
        """
        errors, parent_link_name = joint.resolve_parent_link()
        self._handle_errors(errors)
        return parent_link_name

    def resolve_child_link_name(self, joint):
        """
        Resolves the child link name of an SDFormat Joint.
        :param sdformat.Joint joint: The Joint object.
        :return: The resolved name of the child link.
        :rtype: str
        :raises RuntimeError: if an error is encountered when resolving the
        the link.
        """
        errors, child_link_name = joint.resolve_child_link()
        self._handle_errors(errors)
        return child_link_name

    def _handle_errors(self, errors):
        if errors:
            raise RuntimeError("\n".join(str(err) for err in errors))


class GraphResolver:
    """Forwards all pose and frame resolution calls to a chosen graph resolver
    implementation. By default it will use GraphResolverImpl, this can be
    overridden for testing purposes."""

    def __init__(self, resolver=GraphResolverImpl()):
        """
        Initialize resolver with the provided graph resolver.
        :param GraphResolverImplBase resolver: An implementation of
        GraphResolverImplBase.
        """
        self.resolver = resolver

    def __getattr__(self, attr):
        """Forward all calls too the graph resolver implementation.
        :param attr: Name of method or attribute.
        """
        return getattr(self.resolver, attr)


# This is the default graph resolver, but it can be overridden by tests to
# bypass errors due to frame graphs not being initialized.
graph_resolver = GraphResolver()
