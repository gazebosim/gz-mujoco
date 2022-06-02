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

from ignition.math import Quaterniond, Vector3d, Pose3d

import logging

import math

import sdformat as sdf
import sdformat_mjcf_utils.sdf_utils as su

CAMERA_INDEX = 0


def mjcf_camera_sensor_to_sdf(sensor):
    """
    Converts a camera sensor from MJCF to SDFormat.
    :return: The newly created SDFormat camera sensors.
    :rtype: [sdf.Sensor]
    """
    # camera following a target is not supported
    if sensor.target is not None:
        logging.warning("Not able to process 'target' attribute in camera "
                        "sensor.")
        return None
    sensor_sdf = sdf.Sensor()
    if sensor.name is not None:
        sensor_sdf.set_name(sensor.name)
    else:
        global CAMERA_INDEX
        sensor_sdf.set_name("unmanedcamera_" + str(CAMERA_INDEX))
        CAMERA_INDEX = CAMERA_INDEX + 1

    sensor_sdf.set_type(sdf.Sensortype.CAMERA)
    # The orientation of the camera in Mujoco:
    # The camera is looking along the -Z axis of its frame.
    # The +X axis points to the right
    # the +Y axis points up
    # We need this transform to the camera pose in the SDFormat coordinates
    adapt_pose = Pose3d(Vector3d(), Quaterniond(-math.pi / 2, math.pi / 2, 0))
    pose = su.get_pose_from_mjcf(sensor) * adapt_pose
    sensor_sdf.set_raw_pose(pose)
    camera = sdf.Camera()
    sensor_sdf.set_camera_sensor(camera)
    return sensor_sdf
