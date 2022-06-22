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

from dm_control import mjcf
from dm_control import mujoco

from ignition.math import Vector3d

from sdformat_mjcf.mjcf_to_sdformat.converters import sensor as sensor_conv
from sdformat_mjcf.mjcf_to_sdformat.converters.world import (
    mjcf_worldbody_to_sdf,
)

import sdformat as sdf


from tests.helpers import get_resources_dir

TEST_RESOURCES_DIR = get_resources_dir()


class SensorTest(unittest.TestCase):

    def test_sensor(self):
        mujoco = mjcf.RootElement(model="test")
        body = mujoco.worldbody.add('body')
        camera = body.add('camera')

        sensor_sdf = sensor_conv.mjcf_camera_sensor_to_sdf(camera)
        self.assertEqual("unnamedcamera_2", sensor_sdf.name())
        self.assertEqual(sdf.Sensortype.CAMERA, sensor_sdf.type())
        self.assertNotEqual(None, sensor_sdf.camera_sensor())
        self.assertEqual(Vector3d(), sensor_sdf.raw_pose().pos())
        self.assertEqual(Vector3d(-1.57, 1.57, 0),
                         sensor_sdf.raw_pose().rot().euler())

    def test_sensor_from_file(self):
        mjcf_model = mjcf.from_path(
            str(TEST_RESOURCES_DIR / "test_mujoco.xml"))

        sensor_sdf = sensor_conv.mjcf_camera_sensor_to_sdf(
            mjcf_model.worldbody.camera[0])
        self.assertEqual("fixed", sensor_sdf.name())
        self.assertEqual(sdf.Sensortype.CAMERA, sensor_sdf.type())
        self.assertNotEqual(None, sensor_sdf.camera_sensor())
        self.assertEqual(Vector3d(0, -6, 2), sensor_sdf.raw_pose().pos())
        self.assertEqual(Vector3d(0, 0, 1.57),
                         sensor_sdf.raw_pose().rot().euler())

    def test_sensor_worldbody_to_sdf(self):
        filename = str(TEST_RESOURCES_DIR / "test_mujoco.xml")
        mjcf_model = mjcf.from_path(filename)
        physics = mujoco.Physics.from_xml_path(filename)

        world = sdf.World()
        world.set_name("default")

        mjcf_worldbody_to_sdf(mjcf_model, physics, world, True)

        self.assertEqual(2, world.model_count())
        model = world.model_by_index(0)
        link = model.link_by_index(0)
        self.assertEqual(1, link.sensor_count())

        self.assertEqual("fixed", link.sensor_by_index(0).name())

        model = world.model_by_index(1)
        self.assertNotEqual(None, model)
        link = model.link_by_index(0)
        self.assertEqual(2, link.sensor_count())
        self.assertEqual("unnamedcamera_3", link.sensor_by_index(0).name())
        self.assertEqual("unnamedcamera_4", link.sensor_by_index(1).name())

        self.assertEqual(7, len(world.plugins()))
        self.assertEqual("gz::sim::systems::Physics",
                         world.plugins()[0].name())
        self.assertEqual("ignition-gazebo-physics-system",
                         world.plugins()[0].filename())
        self.assertEqual("gz::sim::systems::Sensors",
                         world.plugins()[1].name())
        self.assertEqual("ignition-gazebo-sensors-system",
                         world.plugins()[1].filename())
        self.assertEqual("gz::sim::systems::UserCommands",
                         world.plugins()[2].name())
        self.assertEqual("ignition-gazebo-user-commands-system",
                         world.plugins()[2].filename())
        self.assertEqual("gz::sim::systems::SceneBroadcaster",
                         world.plugins()[3].name())
        self.assertEqual("ignition-gazebo-scene-broadcaster-system",
                         world.plugins()[3].filename())
        self.assertEqual("gz::sim::systems::ForceTorque",
                         world.plugins()[4].name())
        self.assertEqual("ignition-gazebo-forcetorque-system",
                         world.plugins()[4].filename())
        self.assertEqual("gz::sim::systems::Altimeter",
                         world.plugins()[5].name())
        self.assertEqual("ignition-gazebo-altimeter-system",
                         world.plugins()[5].filename())
        self.assertEqual("gz::sim::systems::Imu",
                         world.plugins()[6].name())
        self.assertEqual("ignition-gazebo-imu-system",
                         world.plugins()[6].filename())


if __name__ == "__main__":
    unittest.main()
