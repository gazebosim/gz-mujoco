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

"""Module to convert joints from SDFormat to MJCF"""

import math

import sdformat as sdf

import sdformat_mjcf_utils.sdf_utils as su

JOINT_DEFAULT_UPPER_LIMIT = 1e16
JOINT_DEFAULT_LOWER_LIMIT = -1e16
JOINT_DEFAULT_DAMPING = 0.0
JOINT_DEFAULT_FRICTION = 0.0
JOINT_DEFAULT_SPRING_REFERENCE = 0.0
JOINT_DEFAULT_SPRING_STIFFNESS = 0.0


def _compute_joint_axis(joint_axis, joint_pose):
    """
    Compute the joint axis unit vector taking into account the pose of the
    joint.

    :param sdformat.JointAxis: The input joint axis
    :param ignition.math.Pose3d joint_pose: The pose of the joint that contains
    the joint axis.
    :param axis_xyz_resolver: Function to resolve the unit vector a joint axis.
    :return: The computed axis unit vector.
    :rtype: list[float]
    """
    xyz_vec = su.graph_resolver.resolve_axis_xyz(joint_axis)
    # Orientation cannot be set on MJCF joints, so resolve the axis
    # unit vector to be in the frame of the child link of the joint. We
    # do this by first resolving the unit vector to the joint frame,
    # and then rotating it by the rotation component of the SDFormat
    # joint. Technically, we can directly resolve the unit vector to
    # the child link, but doing so requires having a working frame
    # graph, which makes testing more complicated.
    xyz_vec_in_child_link = joint_pose.rot().rotate_vector(xyz_vec)
    return su.vec3d_to_list(xyz_vec_in_child_link)


def add_joint(body, joint):
    """
    Converts a joint from SDFormat to MJCF and add it to the given body.

    :param mjcf.Element body: The MJCF body to which the geom is added.
    :param sdformat.Joint joint: The joint to be converted. This would be
    `None` when creating an MJCF freejoint.
    :return: The newly created MJCF joint.
    :rtype: mjcf.Element
    """
    if joint is None:
        return body.add("freejoint")
    elif joint.type() == sdf.JointType.FIXED:
        return None
    elif joint.type() in [
            sdf.JointType.CONTINUOUS,
            sdf.JointType.REVOLUTE,
            sdf.JointType.PRISMATIC
    ]:
        pose = su.graph_resolver.resolve_pose(joint.semantic_pose())
        unique_name = su.find_unique_name(body, "joint", joint.name())
        mjcf_joint = body.add("joint", name=unique_name)
        mjcf_joint.pos = su.vec3d_to_list(pose.pos())
        joint_axis = joint.axis(0)
        if joint_axis is None:
            raise RuntimeError(
                f"Joint type is {joint.type()}, but no axis defined")

        def add_limits(convert_value=lambda x: x):
            range = [JOINT_DEFAULT_LOWER_LIMIT, JOINT_DEFAULT_UPPER_LIMIT]
            is_limited = False
            if joint_axis.lower() != JOINT_DEFAULT_LOWER_LIMIT:
                range[0] = convert_value(joint_axis.lower())
                is_limited = True
            if joint_axis.upper() != JOINT_DEFAULT_UPPER_LIMIT:
                range[1] = convert_value(joint_axis.upper())
                is_limited = True

            if is_limited:
                mjcf_joint.limited = True
                mjcf_joint.range = range

        def add_dynamics(convert_value=lambda x: x):
            if joint_axis.damping() != JOINT_DEFAULT_DAMPING:
                mjcf_joint.damping = joint_axis.damping()
            if joint_axis.friction() != JOINT_DEFAULT_FRICTION:
                mjcf_joint.frictionloss = joint_axis.friction()
            if joint_axis.spring_stiffness() != JOINT_DEFAULT_SPRING_STIFFNESS:
                mjcf_joint.stiffness = joint_axis.spring_stiffness()
            if joint_axis.spring_reference() != JOINT_DEFAULT_SPRING_REFERENCE:
                mjcf_joint.springref = convert_value(
                    joint_axis.spring_reference())

        if joint.type() in [sdf.JointType.CONTINUOUS, sdf.JointType.REVOLUTE]:
            mjcf_joint.type = "hinge"
            add_dynamics(math.degrees)
            if joint.type() == sdf.JointType.REVOLUTE:
                add_limits(math.degrees)
        else:
            mjcf_joint.type = "slide"
            add_limits()
            add_dynamics()

        mjcf_joint.axis = _compute_joint_axis(joint_axis, pose)

        return mjcf_joint
    elif joint.type() == sdf.JointType.BALL:
        pose = su.graph_resolver.resolve_pose(joint.semantic_pose())
        mjcf_joint = body.add("joint", name=joint.name())
        mjcf_joint.pos = su.vec3d_to_list(pose.pos())
        mjcf_joint.type = "ball"
        return mjcf_joint
    else:
        raise RuntimeError(f"Unsupported joint type: {joint.type()}")
