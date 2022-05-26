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

import sdformat as sdf
from ignition.math import Vector3d
from dm_control import mjcf
from mjcf_to_sdformat.converters import sensor as sensor_conv

from tests.helpers import TEST_RESOURCES_DIR


class SensorTest(unittest.TestCase):

    def test_sensor(self):
        mujoco = mjcf.RootElement(model="test")
        body = mujoco.worldbody.add('body')
        camera = body.add('camera')

        sensor_sdf = sensor_conv.mjcf_camera_sensor_to_sdf(camera)
        self.assertEqual("camera", sensor_sdf.name())
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


if __name__ == "__main__":
    unittest.main()
