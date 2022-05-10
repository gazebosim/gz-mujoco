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

"""Test helpers"""

from sdformat_mjcf import sdf_utils as su


def nonthrowing_pose_resolver(sem_pose, relative_to=None):
    """
    Resolves SDFormat poses from a SemanticPose object but instead of raising
    an exception when there's an error, it returns the raw pose of the
    SemanticPose object. This is useful for testing converters with
    programmatically created SDFormat objects that do not contain frame graphs.

    :param sdformat.SemanticPose sem_pose: The SemanticPose object to be
    resolved
    :param str relative_to: (Optional) The frame relative to which the pose is
    resolved.
    :return: The resolved pose if it succeeds, otherwise, the raw pose of
    sem_pose
    :rtype: ignition.math.Pose3d
    """
    try:
        return su.pose_resolver(sem_pose, relative_to)
    except RuntimeError:
        return sem_pose.raw_pose()


def nonthrowing_axis_xyz_resolver(joint_axis):
    """
    Resolves the xyz unit vector of SDFormat Joint axes. If an error is
    encountered, this simply returns the raw value of the xyz vector.
    :param sdformat.JointAxis joint_axis: The JointAxis object to be resolved.
    This is useful for testing converters with programmatically created
    SDFormat objects that do not contain frame graphs.
    :return: The resolved xyz vector.
    :rtype: ignition.math.Vector3d
    :raises RuntimeError: if an error is encountered when resolving the vector.
    """
    try:
        return su.axis_xyz_resolver(joint_axis)
    except RuntimeError:
        return joint_axis.xyz()
