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


def vec3d_to_list(vec):
    """Convert a Vector3d object to a list.
    :param ignition.math.Vector3d vec: Vector3d object to be converted
    :return: List of values of the x, y, and z components of `vec`
    respectively.
    :rtype: list[float]
    """
    return [vec.x(), vec.y(), vec.z()]


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


def pose_resolver(sem_pose, relative_to=None):
    """
    Resolves SDFormat poses from a SemanticPose object.
    :param sdformat.SemanticPose sem_pose: The SemanticPose object to be
    resolved.
    :param str relative_to: (Optional) The frame relative to which the pose is
    resolved.
    :return: The resolved pose.
    :rtype: ignition.math.Pose3d
    :raises RuntimeError: if an error is encountered when resolving the pose.
    """
    pose = Pose3d()
    if relative_to is None:
        errors = sem_pose.resolve(pose)
    else:
        errors = sem_pose.resolve(pose, relative_to)
    if errors:
        raise RuntimeError("\n".join(str(err) for err in errors))
    return pose


def axis_xyz_resolver(joint_axis):
    """
    Resolves the xyz unit vector of SDFormat Joint axes.
    :param sdformat.JointAxis joint_axis: The JointAxis object to be resolved.
    :return: The resolved xyz vector.
    :rtype: ignition.math.Vector3d
    :raises RuntimeError: if an error is encountered when resolving the vector.
    """
    xyz_vec = Vector3d()
    errors = joint_axis.resolve_xyz(xyz_vec)
    if errors:
        raise RuntimeError("\n".join(str(err) for err in errors))
    return xyz_vec
