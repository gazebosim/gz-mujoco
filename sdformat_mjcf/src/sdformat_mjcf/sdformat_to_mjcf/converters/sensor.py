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
import sdformat as sdf
import sdformat_mjcf.utils.sdf_utils as su


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

    if sensor.camera_sensor() is not None:
        # Camera sensors don't map directly to MJCF sensors, instead we map
        # them to MJCF cameras rigidly attached to bodies.
        return _add_camera_sensor(body,
                                  sensor.camera_sensor(),
                                  sensor.name(),
                                  pose)
    else:
        # Creates a site inside the body with the same name as sensor. This
        # site will be used to attach the actual sensor to the body.
        site_unique_name = su.find_unique_name(body, "site", sensor.name())
        body.add("site",
                 name=site_unique_name,
                 pos=su.vec3d_to_list(pose.pos()),
                 euler=su.quat_to_euler_list(pose.rot()))

        if sensor.imu_sensor() is not None:
            return _add_imu(body.root.sensor,
                            sensor.imu_sensor(),
                            site_unique_name)
        elif sensor.altimeter_sensor() is not None:
            return _add_altimeter(body.root.sensor,
                                  sensor.altimeter_sensor(),
                                  site_unique_name)
        elif sensor.force_torque_sensor() is not None:
            return _add_force_torque(body.root.sensor,
                                     sensor.force_torque_sensor(),
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


class NoiseParameterMismatch(Exception):
    pass


def _convert_noise(sensor, component, sensor_name):
    noises = [getattr(sensor, f"{component}_{axis}_noise")() for axis in "xyz"]

    # TODO (azeey) Warn about unsupported noise parameters
    # Check that all the noise parameters are the same for x, y, z axes.
    if not _check_noise_equality(noises):
        logging.warning(
            f"Noise parameter mismatch for {component} of sensor named "
            f"{sensor_name}. Conversion is supported only if the noise "
            "parameters are identical.")
    else:
        return noises[0].std_dev()


def _add_altimeter(sensor, altimeter_sensor, sensor_name):
    """
    Converts an Altimeter sensor from SDFormat to MJCF and add it to the given
    MJCF sensor.

    :param mjcf.Element sensor: The MJCF sensor to which the altimeter is added
    :param sdformat.Altimeter altimeter_sensor: The SDFormat altimeter Sensor
    to be converted.
    :param str altimeter_sensor: The Name of the SDFormat <sensor> element that
    contains the altimeter.
    :return: Converted altimeter element.
    :rtype: [mjcf.Element]
    """
    altimeter_unique_name = su.find_unique_name(sensor, "sensor",
                                                f"altimeter_{sensor_name}")
    frame_pos = sensor.add("framepos",
                           objtype="site",
                           objname=sensor_name,
                           name=altimeter_unique_name)
    frame_pos.noise = altimeter_sensor.vertical_position_noise().std_dev()

    return frame_pos


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

    accel.noise = _convert_noise(imu, "linear_acceleration", sensor_name)

    gyro = sensor.add("gyro", site=sensor_name, name=gyro_unique_name)
    gyro.noise = _convert_noise(imu, "angular_velocity", sensor_name)
    return [accel, gyro]


def _add_force_torque(sensor, ft_sensor, sensor_name):
    """
    Converts a Force Torque sensor from SDFormat to MJCF and add it to the
    given MJCF sensor.

    :param mjcf.Element sensor: The MJCF sensor to which the Force/Torque
    Sensor is added.
    :param sdformat.ForceTorque ft_sensor: The SDFormat Force/Torque Sensor to
    be converted.
    :param str sensor_name: The Name of the SDFormat <sensor> element that
    contains the Force/Torque Sensor.
    :return: Converted force and torque sensor elements.
    :rtype: [mjcf.Element]
    """

    # Note <frame> is handled in add_joint
    meas_dir = ft_sensor.measure_direction()
    if meas_dir != sdf.ForceTorqueMeasureDirection.CHILD_TO_PARENT:
        logging.warning(
            "Only 'child_to_parent' is supported in <measure_direction> of "
            f"Force/Torque sensor named {sensor_name}.")

    force_unique_name = su.find_unique_name(sensor, "sensor",
                                            f"force_{sensor_name}")
    torque_unique_name = su.find_unique_name(sensor, "sensor",
                                             f"torque_{sensor_name}")

    force_sensor = sensor.add("force",
                              site=sensor_name,
                              name=force_unique_name)
    force_sensor.noise = _convert_noise(ft_sensor, "force", sensor_name)

    torque_sensor = sensor.add("torque",
                               site=sensor_name,
                               name=torque_unique_name)
    torque_sensor.noise = _convert_noise(ft_sensor, "torque", sensor_name)

    return [force_sensor, torque_sensor]


def _add_camera_sensor(body, camera_sensor, sensor_name, sensor_pose):
    """
    Converts a Camera sensor from SDFormat to MJCF and adds it to the given
    MJCF body.

    :param mjcf.Element body: The MJCF body to which the Camera is added.
    :param sdformat.Camera camera_sensor: The SDFormat Camera Sensor to be
    converted.
    :param str sensor_name: The name of the SDFormat <sensor> element that
    contains the Camera Sensor.
    :param str sensor_pose: The pose of the SDFormat <sensor> element that
    contains the Camera Sensor.
    :return: Converted camera element.
    :rtype: [mjcf.Element]
    """

    camera_name = su.find_unique_name(body, "camera", sensor_name)
    # The SDFormat camera sensor has a lot of parameters that cannot be mapped
    # to MJCF.
    logging.warning(
        "Converting an SDFormat Camera sensor. Beware that the conversion is "
        "very rudimentary and most SDFormat parameters are not mapped to MJCF."
    )
    mj_camera = body.add("camera",
                         name=camera_name,
                         pos=su.vec3d_to_list(sensor_pose.pos()),
                         euler=su.quat_to_euler_list(sensor_pose.rot()))
    return mj_camera
