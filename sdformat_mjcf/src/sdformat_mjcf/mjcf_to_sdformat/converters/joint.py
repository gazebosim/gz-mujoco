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

from ignition.math import Vector3d
import logging
import math
import sdformat as sdf
import sdformat_mjcf.utils.sdf_utils as su


def add_fixed_joint(parent_name, child_name):
    """
    Return an SDFormat fixed joint
    :param str parent_name: Name of the parent
    :param str child_name: Name of the child
    :return: An SDFormat fixed joint
    :rtype: sdf.Joint
    """
    joint_sdf = sdf.Joint()
    joint_sdf.set_type(sdf.JointType.FIXED)
    if parent_name is None:
        joint_sdf.set_parent_name("world")
    else:
        joint_sdf.set_parent_name(parent_name)
    joint_sdf.set_child_name(child_name)
    joint_sdf.set_name(
        "fixed_" + joint_sdf.parent_name() + "_" + child_name + "_joint")
    return joint_sdf


def mjcf_joint_to_sdf(joint, parent_name, child_name, default_classes=None):
    """
    Convert a MJCF joint in an SDFormat joiint
    :param mjcf.Element joint: MCJF joint
    :param str parent_name: Name of the parent
    :param str child_name: Name of the child
    :return: The newly created SDFormat joint.
    :rtype: sdf.Joint
    """
    try:
        joint.get_attributes()["type"]
    except KeyError:
        joint.set_attributes(**{"type": "hinge"})

    joint_sdf = sdf.Joint()
    joint_axis_sdf = sdf.JointAxis()
    # default value
    joint_axis_sdf.set_xyz(Vector3d(0, 0, 1))

    joint_sdf.set_raw_pose(su.get_pose_from_mjcf(joint))

    if joint.stiffness is not None:
        joint_axis_sdf.set_spring_stiffness(joint.stiffness)
    else:
        joint_axis_sdf.set_spring_stiffness(0)

    if joint.springref is not None:
        if joint.root.compiler.angle == "degree" or \
           joint.root.compiler.angle is None:
            joint_axis_sdf.set_spring_reference(math.radians(joint.springref))
        else:
            joint_axis_sdf.set_spring_reference(joint.springref)
    else:
        joint_axis_sdf.set_spring_reference(0)

    if joint.frictionloss is not None:
        joint_axis_sdf.set_friction(joint.frictionloss)
    else:
        joint_axis_sdf.set_friction(0)

    if joint.limited is not None:
        if joint.limited == "true":
            if joint.range is not None:
                if joint.root.compiler.angle == "degree" or \
                   joint.root.compiler.angle is None:
                    joint_axis_sdf.set_lower(math.radians(joint.range[0]))
                    joint_axis_sdf.set_upper(math.radians(joint.range[1]))
                else:
                    joint_axis_sdf.set_lower(joint.range[0])
                    joint_axis_sdf.set_upper(joint.range[1])
            else:
                joint_axis_sdf.set_lower(0)
                joint_axis_sdf.set_upper(0)

    if joint.damping is not None:
        joint_axis_sdf.set_damping(joint.damping)
    else:
        joint_axis_sdf.set_damping(0)

    if joint.name is not None:
        joint_sdf.set_name("joint_" + joint.name)
    else:
        joint_sdf.set_name(
            "joint_" + joint_sdf.parent_name() + "_" + child_name)

    if parent_name is None:
        joint_sdf.set_parent_name("world")
    else:
        joint_sdf.set_parent_name(parent_name)

    joint_sdf.set_child_name(child_name)

    if joint.axis is not None:
        joint_axis_sdf.set_xyz(Vector3d(joint.axis[0],
                                        joint.axis[1],
                                        joint.axis[2]))
    joint_sdf.set_axis(0, joint_axis_sdf)
    if joint.type == "hinge":
        joint_sdf.set_type(sdf.JointType.REVOLUTE)
        return joint_sdf
    if joint.type == "slide":
        joint_sdf.set_type(sdf.JointType.PRISMATIC)
        return joint_sdf
    else:
        logging.warning(f"Not able to process this type of joint "
                        f"{joint.type}.")
        return None
