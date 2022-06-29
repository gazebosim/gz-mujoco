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

"""Utility functions that aid in conversion between SDFormat and MJCF"""

import logging
import math
from ignition.math import Color, Pose3d, Quaterniond, Vector3d, Matrix3d

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


def get_rotation(element):
    """Get the angles from a MJCF element
    :param mjcf.Element element: Element to extract the angles
    :rtype: ignition.math.Quateriond
    """
    angle_type = "degree"
    if element.root.compiler.angle is not None:
        angle_type = element.root.compiler.angle
    quat = Quaterniond()
    if element.euler is not None:
        eulerseq = "xyz"
        if element.root.compiler.eulerseq is not None:
            eulerseq = element.root.compiler.eulerseq

        result = list_to_vec3d(element.euler)
        if angle_type == "degree":
            result = result * math.pi / 180.0

        if eulerseq == "XYZ":
            quat = Quaterniond(result)
        elif eulerseq == "xyz":
            quat = Quaterniond(result.x(), 0, 0) * Quaterniond(
                0, result.y(), 0) * Quaterniond(0, 0, result.z())
        else:
            raise RuntimeError(f"Unsupported Euler sequence {eulerseq}. Euler "
                               "angles will be set to identity")

    elif element.zaxis is not None:
        z = Vector3d(0, 0, 1)
        quat = Quaterniond()
        quat.set_from_2_axes(z, list_to_vec3d(element.zaxis))
    elif element.quat is not None:
        quat = wxyz_list_to_quat(element.quat)
    elif element.xyaxes is not None:
        v1 = list_to_vec3d(element.xyaxes)
        v2 = list_to_vec3d(element.xyaxes[-3:])
        z = v1.cross(v2)
        d = v1.dot(v2)
        v2 = Vector3d(v2.x() - v1.x() * d,
                      v2.y() - v1.y() * d,
                      v2.z() - v1.z() * d)
        mat = Matrix3d(v1.x(), v2.x(), z.x(),
                       v1.y(), v2.y(), z.y(),
                       v1.z(), v2.z(), z.z())

        quat = Quaterniond(mat)
    return quat


def vec2d_to_list(vec):
    """
    Convert a Vector2d object to a list.
    :param ignition.math.Vector2d vec: Vector2d object to be converted.
    :return: List of values of the x, and y components of `vec` respectively.
    :rtype: list[float]
    """
    return [vec.x(), vec.y()]


def euler_list_to_quat(list):
    """
    Convert a euler list to a Quaternion
    :param List of values of the roll, pitch, and yaw components of `vec`
    respectively.
    :return: The newly created Quaterniond
    :rtype: ignition.math.Quaterniond
    """
    return Quaterniond(Vector3d(list[0], list[1], list[2]))


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


def wxyz_list_to_quat(quat):
    """
    Convert a Quaternion list defined as wxyz to a gz.math.Quateriond.
    :param gz.math.Quaterniond quat: Quaternion defined as a list.
    :return: Return a Quaterniond
    :rtype: gz.math.Quaterniond
    """
    return Quaterniond(quat[0], quat[1], quat[2], quat[3])


def rgba_to_color(rgba):
    """
    Convert a MJCF rgba color list to a ignition math Color
    :param list rgba: List of color (red, green, blue, alpha).
    :return: The newly created color
    :rtype: ignition.math.color
    """
    return Color(rgba[0], rgba[1], rgba[2], rgba[3])


def get_pose_from_mjcf(element):
    """
    Get the pose from a MJCF element
    :param mjcf.Element element: MJCF element to get the pose
    :return: The newly created Pose3d
    :rtype: ignition.math.Pose3d
    """
    pos = [0, 0, 0]
    quat = Quaterniond()
    try:
        if element.pos is not None:
            pos = element.pos
        quat = get_rotation(element)
    except AttributeError:
        pass
    return Pose3d(list_to_vec3d(pos), quat)


def prefix_name_with_index(prefix, name, index):
    """
    Create a new string a given `name` with `prefix` and an `index`
    :param str prefix: The prefix. For example: visual or collision
    :param str name: Name to prefix.
    :param str index: Index to add as a suffix to avoid name collision
    :return: The newly created name
    :rtype: str
    """
    if name is not None:
        return prefix + NAME_DELIMITER + name
    else:
        new_name = prefix + NAME_DELIMITER + str(index)
        return new_name


def find_unique_name(elem, namespace, name):
    """
    Find a unique name in the given `namespace` based on the provided `name`.
    If the provided name is found in the namespace, a numeric suffix is added
    to the name. This is again tested for uniqueness and the suffix is
    incremented until a unique name is found.

    For example, if we want to add a geom to a body and need to find a unique
    name for it where the starting name is "bumper", we would call this
    function as:

        unique_name = find_unique_name(body, "geom", "bumper")

    where `body` is the MJCF body to which the geom is being added. If the body
    already has "bumper", the returned name would be "bumper_0".


    :param mjcf.Element elem: The element whose namescope is used for checking
    name uniqueness.
    :param str namespace: The namespace (eg. "body", "geom") in the namescope
    in which the name is searched.
    :param str name: Starting name to be checked for uniqueness. This will be
    the prefix of the returned name.
    :return: The newly created name
    :rtype: str
    """
    index = 0
    test_name = name
    while elem.namescope.has_identifier(namespace, test_name):
        test_name = f"{name}{NAME_DELIMITER}{index}"
        index += 1
    return test_name


def get_asset_filename_on_disk(asset):
    """
    This method returns the name of the asset file without the hash.
    :param mjcf.Asset asset: The asset from which the filename is recovered.
    :return: The name of the file on disk.
    :rtype: str
    """
    file = asset.file
    if file.prefix:
        return file.prefix + file.extension
    else:
        logging.warning("Asset file does not exist on disk")
        return file.get_vfs_filename()


def get_value_or_default(val, default_val):
    """
    Returns the value of `val` if it's not None, otherwise returns the passed
    in default value.
    :param any val: Any variable that maybe None
    :param any default_val: The default value.
    :return: The value or it's not None, otherwise the default value
    :rtype: any
    """
    if val is None:
        return default_val
    return val


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
        :raises SDFErrorsException: if an error is encountered when resolving
        the pose.
        """
        if relative_to is None:
            return sem_pose.resolve()
        else:
            return sem_pose.resolve(relative_to)

    def resolve_axis_xyz(self, joint_axis):
        """
        Resolves the xyz unit vector of SDFormat Joint axes.
        :param sdformat.JointAxis joint_axis: The JointAxis object to be
        resolved.
        :return: The resolved xyz vector.
        :rtype: ignition.math.Vector3d
        :raises SDFErrorsException: if an error is encountered when resolving
        the vector.
        """
        return joint_axis.resolve_xyz()

    def resolve_parent_link_name(self, joint):
        """
        Resolves the parent link name of an SDFormat Joint.
        :param sdformat.Joint joint: The Joint object.
        :return: The resolved name of the parent link.
        :rtype: str
        :raises SDFErrorsException: if an error is encountered when resolving
        the the link.
        """
        return joint.resolve_parent_link()

    def resolve_child_link_name(self, joint):
        """
        Resolves the child link name of an SDFormat Joint.
        :param sdformat.Joint joint: The Joint object.
        :return: The resolved name of the child link.
        :rtype: str
        :raises SDFErrorsException: if an error is encountered when resolving
        the the link.
        """
        return joint.resolve_child_link()


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
