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

import unittest
from numpy.testing import assert_allclose
from math import pi

import sdformat as sdf
from ignition.math import Pose3d
from dm_control import mjcf

from sdformat_to_mjcf.converters.sensor import add_sensor
from tests import helpers


class ImuSensorTest(helpers.TestCase):
    test_pose = Pose3d(1, 2, 3, pi / 2, pi / 3, pi / 4)
    expected_pos = [1.0, 2.0, 3.0]
    expected_euler = [90.0, 60.0, 45.0]

    def setUp(self):
        self.mujoco = mjcf.RootElement(model="test")
        self.body = self.mujoco.worldbody.add("body", name="test_body")
        self.sensor = sdf.Sensor()
        self.sensor.set_name("imu_sensor")
        self.sensor.set_type(sdf.Sensortype.IMU)
        self.sensor.set_raw_pose(self.test_pose)

    def test_default(self):
        imu = sdf.IMU()
        self.sensor.set_imu_sensor(imu)
        mjcf_sensors = add_sensor(self.body, self.sensor)

        self.assertIsNotNone(mjcf_sensors)
        self.assertEqual(2, len(mjcf_sensors))
        self.assertIsNotNone(self.mujoco.sensor)

        mjcf_accels = self.mujoco.sensor.get_children("accelerometer")
        self.assertEqual(1, len(mjcf_accels))
        mjcf_gyros = self.mujoco.sensor.get_children("gyro")
        self.assertEqual(1, len(mjcf_gyros))

        sensor_sites = self.body.get_children("site")
        self.assertEqual(1, len(sensor_sites))
        self.assertEqual(self.sensor.name(), sensor_sites[0].name)
        assert_allclose(self.expected_pos, sensor_sites[0].pos)
        assert_allclose(self.expected_euler, sensor_sites[0].euler)

        mjcf_accel, mjcf_gyro = mjcf_sensors
        self.assertAlmostEqual(f"accelerometer_{self.sensor.name()}",
                               mjcf_accel.name)
        self.assertAlmostEqual(self.sensor.name(), mjcf_accel.site)

        self.assertAlmostEqual(f"gyro_{self.sensor.name()}", mjcf_gyro.name)
        self.assertAlmostEqual(self.sensor.name(), mjcf_gyro.site)

    def test_noise(self):
        imu = sdf.IMU()
        accel_noise = sdf.Noise()
        accel_noise.set_type(sdf.NoiseType.GAUSSIAN)
        accel_noise.set_std_dev(0.2)

        gyro_noise = sdf.Noise()
        gyro_noise.set_type(sdf.NoiseType.GAUSSIAN)
        gyro_noise.set_std_dev(0.02)

        imu.set_linear_acceleration_x_noise(accel_noise)
        imu.set_linear_acceleration_y_noise(accel_noise)
        imu.set_linear_acceleration_z_noise(accel_noise)

        imu.set_angular_velocity_x_noise(gyro_noise)
        imu.set_angular_velocity_y_noise(gyro_noise)
        imu.set_angular_velocity_z_noise(gyro_noise)
        self.sensor.set_imu_sensor(imu)
        mjcf_sensors = add_sensor(self.body, self.sensor)

        self.assertIsNotNone(mjcf_sensors)
        self.assertEqual(2, len(mjcf_sensors))
        self.assertIsNotNone(self.mujoco.sensor)

        mjcf_accels = self.mujoco.sensor.get_children("accelerometer")
        self.assertEqual(1, len(mjcf_accels))
        mjcf_gyros = self.mujoco.sensor.get_children("gyro")
        self.assertEqual(1, len(mjcf_gyros))

        sensor_sites = self.body.get_children("site")
        self.assertEqual(1, len(sensor_sites))
        self.assertEqual(self.sensor.name(), sensor_sites[0].name)
        assert_allclose(self.expected_pos, sensor_sites[0].pos)
        assert_allclose(self.expected_euler, sensor_sites[0].euler)

        mjcf_accel, mjcf_gyro = mjcf_sensors
        self.assertAlmostEqual(f"accelerometer_{self.sensor.name()}",
                               mjcf_accel.name)
        self.assertAlmostEqual(self.sensor.name(), mjcf_accel.site)
        self.assertAlmostEqual(accel_noise.std_dev(), mjcf_accel.noise)

        self.assertAlmostEqual(f"gyro_{self.sensor.name()}",
                               mjcf_gyro.name)
        self.assertAlmostEqual(self.sensor.name(), mjcf_gyro.site)
        self.assertAlmostEqual(gyro_noise.std_dev(), mjcf_gyro.noise)

    def test_noise_mismatch(self):
        imu = sdf.IMU()
        accel_noise = sdf.Noise()
        accel_noise.set_type(sdf.NoiseType.GAUSSIAN)
        accel_noise.set_std_dev(0.2)

        gyro_noise = sdf.Noise()
        gyro_noise.set_type(sdf.NoiseType.GAUSSIAN)
        gyro_noise.set_std_dev(0.02)

        imu.set_linear_acceleration_x_noise(accel_noise)
        imu.set_angular_velocity_x_noise(gyro_noise)

        self.sensor.set_imu_sensor(imu)
        with self.assertLogs(level="WARN") as cm:
            mjcf_sensors = add_sensor(self.body, self.sensor)
        self.assertEqual(2, len(cm.output))
        self.assertIn("Noise parameter mismatch for linear_acceleration",
                      cm.output[0])
        self.assertIn("Noise parameter mismatch for angular_velocity",
                      cm.output[1])

        self.assertIsNotNone(mjcf_sensors)
        self.assertEqual(2, len(mjcf_sensors))
        mjcf_accel, mjcf_gyro = mjcf_sensors
        self.assertIsNone(mjcf_accel.noise)
        self.assertIsNone(mjcf_gyro.noise)


if __name__ == "__main__":
    unittest.main()
