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

import unittest
import math

from ignition.math import Quaterniond, Vector3d

from dm_control import mjcf
from dm_control import mujoco

from mjcf_to_sdformat.converters.world import mjcf_worldbody_to_sdf

import sdformat as sdf

from tests.helpers import TEST_RESOURCES_DIR


class ImuTest(unittest.TestCase):

    def test_imu(self):
        filename = str(TEST_RESOURCES_DIR / "test_imu.xml")
        mjcf_model = mjcf.from_path(filename)
        physics = mujoco.Physics.from_xml_path(filename)

        world = sdf.World()
        world.set_name("default")

        mjcf_worldbody_to_sdf(mjcf_model, physics, world)
        self.assertEqual("default", world.name())
        self.assertEqual(2, world.model_count())

        model = world.model_by_index(1)
        self.assertNotEqual(None, model)
        link = model.link_by_index(1)
        self.assertEqual("link_0", link.name())
        self.assertEqual(2, link.sensor_count())

        sensor = link.sensor_by_index(0)
        self.assertNotEqual(None, sensor)
        imu = sensor.imu_sensor()
        self.assertNotEqual(None, imu)
        self.assertEqual("accelerometer_imu_sensor", sensor.name())
        self.assertEqual(100, sensor.update_rate())
        self.assertEqual(Vector3d(1, 0, 0), sensor.raw_pose().pos())
        self.assertEqual(Vector3d(0, math.pi/4, 0),
                         sensor.raw_pose().rot().euler())
        self.assertEqual(0.1, imu.linear_acceleration_x_noise().std_dev())
        self.assertEqual(0.1, imu.linear_acceleration_y_noise().std_dev())
        self.assertEqual(0.1, imu.linear_acceleration_z_noise().std_dev())
        self.assertEqual(0.1, imu.angular_velocity_x_noise().std_dev())
        self.assertEqual(0.1, imu.angular_velocity_y_noise().std_dev())
        self.assertEqual(0.1, imu.angular_velocity_z_noise().std_dev())

        sensor = link.sensor_by_index(1)
        self.assertNotEqual(None, sensor)
        imu = sensor.imu_sensor()
        self.assertNotEqual(None, imu)
        self.assertEqual("unnamedimu_0", sensor.name())
        self.assertEqual(100, sensor.update_rate())
        self.assertEqual(Vector3d(1, 0, 0), sensor.raw_pose().pos())
        self.assertEqual(Vector3d(0, math.pi/4, 0),
                         sensor.raw_pose().rot().euler())
        self.assertEqual(0.0, imu.linear_acceleration_x_noise().std_dev())
        self.assertEqual(0.0, imu.linear_acceleration_y_noise().std_dev())
        self.assertEqual(0.0, imu.linear_acceleration_z_noise().std_dev())
        self.assertEqual(0.0, imu.angular_velocity_x_noise().std_dev())
        self.assertEqual(0.0, imu.angular_velocity_y_noise().std_dev())
        self.assertEqual(0.0, imu.angular_velocity_z_noise().std_dev())


if __name__ == "__main__":
    unittest.main()
