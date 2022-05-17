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

import sdformat as sdf


def add_mjcf_joint_to_sdf(joint, parent_name, child_name):
    if joint.root.default.joint is not None:
        joint_attributes = joint.get_attributes().items()
        for k, v in joint.root.default.joint.get_attributes().items():
            try:
                joint.get_attributes()[k]
            except:
                joint.set_attributes(**{k:v})
        # joint.set_attributes(**joint.root.default.joint.get_attributes())
    joint_sdf = sdf.Joint()
    joint_axis_sdf = sdf.JointAxis()
    if joint.type == "hinge":
        joint_sdf.set_type(sdf.Joint.JointType.REVOLUTE)
        # joint_sdf.set_raw_pose(Posed3d(Vector3d))
        errors = joint_axis_sdf.set_xyz(Vector3d(joint.axis[0],
                                                 joint.axis[1],
                                                 joint.axis[2]))
        joint_sdf.set_name(joint.name)
        if parent_name is None:
            joint_sdf.set_parent_link_name("world")
        else:
            joint_sdf.set_parent_link_name(parent_name)

        joint_sdf.set_child_link_name(child_name)
        geom = joint.root.find("geom", child_name)
        if geom is not None and geom.fromto is not None:
            v1 = Vector3d(geom.fromto[0], geom.fromto[1], geom.fromto[2])
            v2 = Vector3d(geom.fromto[3], geom.fromto[4], geom.fromto[5])
            joint_sdf.set_raw_pose(Pose3d(Vector3d(-(v2 - v1) / 2),
                                          Quaterniond()))
        if joint.damping is not None:
            joint_axis_sdf.set_damping(joint.damping)
        joint_sdf.set_axis(0, joint_axis_sdf)
        return joint_sdf
    else:
        return None
