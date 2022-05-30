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

from ignition.math import Pose3d, Vector3d, Quaterniond
import math
import sdformat as sdf


def add_fix_joint(parent_name, child_name):
    joint_sdf = sdf.Joint()
    joint_sdf.set_type(sdf.JointType.FIXED)
    if parent_name is None:
        joint_sdf.set_parent_link_name("world")
    else:
        joint_sdf.set_parent_link_name(parent_name)
    joint_sdf.set_child_link_name(child_name)
    joint_sdf.set_name(
        "fixed_" + joint_sdf.parent_link_name() + "_" + joint_sdf.child_link_name() + "_joint")

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
    if joint.root.default.joint is not None:
        for k, v in joint.root.default.joint.get_attributes().items():
            try:
                joint.get_attributes()[k]
            except:
                joint.set_attributes(**{k:v})
    if default_classes is not None:
        for default_class in default_classes:
            if default_class.joint is not None:
                for k, v in default_class.joint.get_attributes().items():
                    try:
                        joint.get_attributes()[k]
                    except:
                        joint.set_attributes(**{k:v})
    if joint.dclass is not None:
        if joint.dclass.joint is not None:
            for k, v in joint.dclass.joint.get_attributes().items():
                try:
                    joint.get_attributes()[k]
                    joint.set_attributes(**{k:v})
                except:
                    joint.set_attributes(**{k:v})

    try:
        joint.get_attributes()["type"]
    except:
        joint.set_attributes(**{"type":"hinge"})

    joint_sdf = sdf.Joint()
    joint_axis_sdf = sdf.JointAxis()
    # default value
    errors = joint_axis_sdf.set_xyz(Vector3d(0, 0, 1))

    if joint.stiffness is not None:
        joint_axis_sdf.set_stiffness(joint.stiffness)
    else:
        joint_axis_sdf.set_stiffness(0)

    if joint.limited is not None:
        if joint.limited == "true":
            if joint.range is not None:
                joint_axis_sdf.set_lower(joint.range[0] * math.pi / 180.0)
                joint_axis_sdf.set_upper(joint.range[1] * math.pi / 180.0)
            else:
                joint_axis_sdf.set_lower(-1)
                joint_axis_sdf.set_upper(1)
    if joint.damping is not None:
        joint_axis_sdf.set_damping(joint.damping)
    else:
        joint_axis_sdf.set_damping(0)

    joint_sdf.set_name(joint.name + "_joint")
    if parent_name is None:
        joint_sdf.set_parent_link_name("world")
    else:
        joint_sdf.set_parent_link_name(parent_name)

    joint_sdf.set_child_link_name(child_name)

    if joint.axis is not None:
        errors = joint_axis_sdf.set_xyz(Vector3d(joint.axis[0],
                                                 joint.axis[1],
                                                 joint.axis[2]))

    if joint.type == "hinge":
        joint_sdf.set_type(sdf.JointType.REVOLUTE)


        joint_sdf.set_axis(0, joint_axis_sdf)
        return joint_sdf
    if joint.type == "slide":
        joint_sdf.set_type(sdf.JointType.PRISMATIC)
        errors = joint_axis_sdf.set_xyz(Vector3d(joint.axis[0],
                                                 joint.axis[1],
                                                 joint.axis[2]))

        joint_sdf.set_axis(0, joint_axis_sdf)
        return joint_sdf
    else:
        print("Not able to process this type of joint ", joint.type)
        return None
