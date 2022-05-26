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

import logging
import sdformat_mjcf_utils.sdf_utils as su


def add_sensor(body, sensor):
    """
    Converts a sensor from SDFormat to MJCF and add it to the given
    body.

    :param mjcf.Element body: The MJCF body to which the sensor is added.
    :param sdformat.Sensor sensor: The SDFormat sensor to be converted.
    :return: One or more of the newly created MJCF sensors. None if there are
    no supported sensors to be converted.
    :rtype: [mjcf.Element]
    """
    sem_pose = sensor.semantic_pose()
    pose = su.graph_resolver.resolve_pose(sem_pose)

    # Creates a site inside the body with the same name as sensor. This site
    # will be used to attach the actual sensor to the body.
    site_unique_name = su.find_unique_name(body, "site", sensor.name())
    body.add("site",
             name=site_unique_name,
             pos=su.vec3d_to_list(pose.pos()),
             euler=su.quat_to_euler_list(pose.rot()))

    if sensor.imu_sensor() is not None:
        return _add_imu(body.root.sensor, sensor.imu_sensor(),
                        site_unique_name)


def _check_noise_equality(noises):
    """
    Check that all noise parameters are identical
    :param [sdformat.Noise] noises: Noise parameters to check
    :return: True if all noise parameters are equal to each other.
    :rtype: bool
    """
    assert len(noises) > 0
    first_item = noises[0]
    return all(x == first_item for x in noises)


def _add_imu(sensor, imu, sensor_name):
    """
    Converts an IMU sensor from SDFormat to MJCF and add it to the given
    MJCF sensor.

    :param mjcf.Element sensor: The MJCF sensor to which the IMU is added.
    :param sdformat.IMU imu: The SDFormat IMU Sensor to be converted.
    :param str sensor_name: The Name of the SDFormat <sensor> element that
    contains the IMU.
    :return: Converted accelerometer and gyro elements.
    :rtype: [mjcf.Element]
    """

    # The following elements in <imu> are not currently supported.
    # - <orientation_reference_frame>
    # - <enable_orientation>
    #
    # For noise parameters, only the <stddev> is suported.
    # TODO (azeey) Warn about unsupported elements in <imu>
    accel_unique_name = su.find_unique_name(sensor, "sensor",
                                            f"accelerometer_{sensor_name}")
    gyro_unique_name = su.find_unique_name(sensor, "sensor",
                                           f"gyro_{sensor_name}")

    accel = sensor.add("accelerometer",
                       site=sensor_name,
                       name=accel_unique_name)

    # TODO (azeey) Warn about unsupported noise parameters
    accel_noises = [
        imu.linear_acceleration_x_noise(),
        imu.linear_acceleration_y_noise(),
        imu.linear_acceleration_z_noise()
    ]
    # Check that all the noise parameters are the same for x, y, z axes.
    if not _check_noise_equality(accel_noises):
        logging.warning(
            "Noise parameter mismatch for linear_acceleration of "
            f"IMU named {sensor_name}. Conversion is supported only if the "
            "noise parameters are identical.")
    else:
        accel.noise = accel_noises[0].std_dev()

    gyro = sensor.add("gyro", site=sensor_name, name=gyro_unique_name)
    gyro_noises = [
        imu.angular_velocity_x_noise(),
        imu.angular_velocity_y_noise(),
        imu.angular_velocity_z_noise()
    ]
    if not _check_noise_equality(gyro_noises):
        logging.warning(
            "Noise parameter mismatch for angular_velocity of "
            f"IMU named {sensor_name}. Conversion is supported only if the "
            "noise parameters are identical.")
    else:
        gyro.noise = gyro_noises[0].std_dev()

    return [accel, gyro]
