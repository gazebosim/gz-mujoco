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
import sdformat_mjcf.utils.sdf_utils as su

CAMERA_INDEX = 0
IMU_INDEX = 0
FORCCE_TORQUE_INDEX = 0


def mjcf_force_torque_sensor_to_sdf(sensor, model):
    """
    Converts a force or torque sensor from MJCF to SDFormat.
    :param mjcf.sensor sensor: The MJCF force or torque sensor to convert
    :param sdf.Model sensor: The MJCF sensor to convert
    :return: The newly created SDFormat IMU sensors.
    :rtype: [sdf.Sensor]
    """
    sensor_sdf = sdf.Sensor()
    if sensor.name is not None:
        sensor_sdf.set_name(sensor.name)
    else:
        global FORCCE_TORQUE_INDEX
        sensor_sdf.set_name("unnamedforcetorque_" + str(FORCCE_TORQUE_INDEX))
        FORCCE_TORQUE_INDEX = FORCCE_TORQUE_INDEX + 1

    sensor_sdf.set_type(sdf.Sensortype.FORCE_TORQUE)
    sensor_sdf.set_update_rate(100)
    sensor_sdf.set_raw_pose(su.get_pose_from_mjcf(sensor.site))
    force_torque = sdf.ForceTorque()

    if sensor.noise is not None:
        noise = sdf.Noise()
        noise.set_type(sdf.NoiseType.GAUSSIAN)
        noise.set_std_dev(sensor.noise)
        force_torque.set_force_x_noise(noise)
        force_torque.set_force_y_noise(noise)
        force_torque.set_force_z_noise(noise)
        force_torque.set_torque_x_noise(noise)
        force_torque.set_torque_y_noise(noise)
        force_torque.set_torque_z_noise(noise)

    sensor_sdf.set_force_torque_sensor(force_torque)

    link = model.link_by_name(sensor.site.parent.name)
    if link is not None:
        link.add_sensor(sensor_sdf)

    return sensor_sdf


def mjcf_accelerometer_gyro_sensor_to_sdf(sensor, model):
    """
    Converts a accelerometer or gyro sensor from MJCF to SDFormat.
    :param mjcf.sensor sensor: The MJCF accelerometer or gyro sensor to convert
    :param sdf.Model sensor: The MJCF sensor to convert
    :return: The newly created SDFormat IMU sensors.
    :rtype: sdf.Sensor
    """
    sensor_sdf = sdf.Sensor()
    if sensor.name is not None:
        sensor_sdf.set_name(sensor.name)
    else:
        global IMU_INDEX
        sensor_sdf.set_name("unnamedimu_" + str(IMU_INDEX))
        IMU_INDEX = IMU_INDEX + 1

    sensor_sdf.set_type(sdf.Sensortype.IMU)
    sensor_sdf.set_update_rate(100)
    sensor_sdf.set_raw_pose(su.get_pose_from_mjcf(sensor.site))
    imu = sdf.IMU()

    if sensor.noise is not None:
        noise = sdf.Noise()
        noise.set_type(sdf.NoiseType.GAUSSIAN)
        noise.set_std_dev(sensor.noise)
        imu.set_linear_acceleration_x_noise(noise)
        imu.set_linear_acceleration_y_noise(noise)
        imu.set_linear_acceleration_z_noise(noise)
        imu.set_angular_velocity_x_noise(noise)
        imu.set_angular_velocity_y_noise(noise)
        imu.set_angular_velocity_z_noise(noise)

    sensor_sdf.set_imu_sensor(imu)

    link = model.link_by_name(sensor.site.parent.name)
    if link is not None:
        link.add_sensor(sensor_sdf)

    return sensor_sdf


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
        sensor_sdf.set_name("unnamedcamera_" + str(CAMERA_INDEX))
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
    sensor_sdf.set_update_rate(25)
    camera = sdf.Camera()
    sensor_sdf.set_camera_sensor(camera)
    return sensor_sdf
