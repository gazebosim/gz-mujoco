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

import math
import unittest

from ignition.math import Vector3d

from dm_control import mjcf
from dm_control import mujoco

from mjcf_to_sdformat.converters.world import mjcf_worldbody_to_sdf

import sdformat as sdf

from tests.helpers import TEST_RESOURCES_DIR


class ForceTorqueTest(unittest.TestCase):

    def test_force_torque(self):
        filename = str(TEST_RESOURCES_DIR / "test_force_torque.xml")
        mjcf_model = mjcf.from_path(filename)
        physics = mujoco.Physics.from_xml_path(filename)

        world = sdf.World()
        world.set_name("default")

        mjcf_worldbody_to_sdf(mjcf_model, physics, world)
        self.assertEqual("default", world.name())
        self.assertEqual(2, world.model_count())

        model = world.model_by_index(1)
        self.assertNotEqual(None, model)
        link = model.link_by_index(0)
        self.assertEqual("link", link.name())
        self.assertEqual(2, link.sensor_count())

        sensor = link.sensor_by_index(0)
        self.assertNotEqual(None, sensor)
        force = sensor.force_torque_sensor()
        self.assertEqual("force_force_torque_sensor", sensor.name())
        self.assertEqual(sdf.Sensortype.FORCE_TORQUE, sensor.type())
        self.assertEqual(100, sensor.update_rate())
        self.assertEqual(Vector3d(10, 11, 12), sensor.raw_pose().pos())
        self.assertEqual(Vector3d(0, 0, math.pi / 4),
                         sensor.raw_pose().rot().euler())
        self.assertEqual(0, force.force_x_noise().std_dev())
        self.assertEqual(0, force.force_y_noise().std_dev())
        self.assertEqual(0, force.force_z_noise().std_dev())
        self.assertEqual(0, force.torque_x_noise().std_dev())
        self.assertEqual(0, force.torque_y_noise().std_dev())
        self.assertEqual(0, force.torque_z_noise().std_dev())

        sensor = link.sensor_by_index(1)
        self.assertNotEqual(None, sensor)
        torque = sensor.force_torque_sensor()
        self.assertEqual("unnamedforcetorque_0", sensor.name())
        self.assertEqual(sdf.Sensortype.FORCE_TORQUE, sensor.type())
        self.assertEqual(100, sensor.update_rate())
        self.assertEqual(Vector3d(10, 11, 12), sensor.raw_pose().pos())
        self.assertEqual(Vector3d(0, 0, math.pi / 4),
                         sensor.raw_pose().rot().euler())
        self.assertEqual(0.0005, torque.force_x_noise().std_dev())
        self.assertEqual(0.0005, torque.force_y_noise().std_dev())
        self.assertEqual(0.0005, torque.force_z_noise().std_dev())
        self.assertEqual(0.0005, torque.torque_x_noise().std_dev())
        self.assertEqual(0.0005, torque.torque_y_noise().std_dev())
        self.assertEqual(0.0005, torque.torque_z_noise().std_dev())


if __name__ == "__main__":
    unittest.main()
