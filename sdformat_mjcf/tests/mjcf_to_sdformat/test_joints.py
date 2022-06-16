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

from ignition.math import Pose3d, Vector3d

from dm_control import mjcf
from dm_control import mujoco

from sdformat_mjcf.mjcf_to_sdformat.converters.world import (
    mjcf_worldbody_to_sdf,
)

import sdformat as sdf

from tests.helpers import get_resources_dir

TEST_RESOURCES_DIR = get_resources_dir()


class JointTest(unittest.TestCase):

    def test_defaults(self):
        filename = str(TEST_RESOURCES_DIR / "test_joints.xml")
        mjcf_model = mjcf.from_path(filename)
        physics = mujoco.Physics.from_xml_path(filename)

        world = sdf.World()
        world.set_name("default")

        mjcf_worldbody_to_sdf(mjcf_model, physics, world)

        self.assertEqual("default", world.name())
        self.assertEqual(2, world.model_count())
        model = world.model_by_index(0)
        self.assertNotEqual(None, model)
        self.assertEqual(1, model.link_count())
        self.assertTrue(model.static())

        model = world.model_by_index(1)
        self.assertNotEqual(None, model)
        self.assertEqual("two-link planar reacher", model.name())

        self.assertEqual(4, model.joint_count())
        joint = model.joint_by_index(0)
        self.assertNotEqual(None, joint)
        self.assertEqual(sdf.JointType.REVOLUTE, joint.type())
        self.assertEqual("world", joint.parent_link_name())
        self.assertEqual("body1", joint.child_link_name())
        self.assertNotEqual(None, joint.axis(0))
        self.assertEqual(Vector3d(0, 0, 1), joint.axis(0).xyz())
        self.assertEqual(0, joint.axis(0).damping())
        self.assertEqual(100000000.0, joint.axis(0).stiffness())
        self.assertEqual(0.0, joint.axis(0).spring_stiffness())
        self.assertEqual(0.0, joint.axis(0).spring_reference())
        self.assertEqual(0.0, joint.axis(0).friction())
        self.assertEqual(Pose3d(), joint.raw_pose())

        joint = model.joint_by_index(1)
        self.assertNotEqual(None, joint)
        self.assertEqual(sdf.JointType.REVOLUTE, joint.type())
        self.assertEqual("world", joint.parent_link_name())
        self.assertEqual("body1", joint.child_link_name())
        self.assertNotEqual(None, joint.axis(0))
        self.assertEqual(Vector3d(0, 1, 0), joint.axis(0).xyz())
        self.assertEqual(0, joint.axis(0).damping())
        self.assertEqual(100000000.0, joint.axis(0).stiffness())
        self.assertEqual(0.0, joint.axis(0).spring_stiffness())
        self.assertEqual(0.019198621771937627,
                         joint.axis(0).spring_reference())
        self.assertEqual(2.0, joint.axis(0).friction())

        joint = model.joint_by_index(2)
        self.assertNotEqual(None, joint)
        self.assertEqual(sdf.JointType.PRISMATIC, joint.type())
        self.assertEqual("body1", joint.parent_link_name())
        self.assertEqual("body2", joint.child_link_name())
        self.assertNotEqual(None, joint.axis(0))
        self.assertEqual(Vector3d(0, 0, 1), joint.axis(0).xyz())
        self.assertEqual(0, joint.axis(0).damping())
        self.assertEqual(100000000.0, joint.axis(0).stiffness())
        self.assertEqual(0.0, joint.axis(0).spring_stiffness())
        self.assertEqual(0.0, joint.axis(0).spring_reference())
        self.assertEqual(0.0, joint.axis(0).friction())

        joint = model.joint_by_index(3)
        self.assertNotEqual(None, joint)
        self.assertEqual(sdf.JointType.PRISMATIC, joint.type())
        self.assertEqual("body2", joint.parent_link_name())
        self.assertEqual("body3", joint.child_link_name())
        self.assertNotEqual(None, joint.axis(0))
        self.assertEqual(Vector3d(1, 0, 0), joint.axis(0).xyz())
        self.assertEqual(0.01, joint.axis(0).damping())
        self.assertEqual(100000000.0, joint.axis(0).stiffness())
        self.assertEqual(10.0, joint.axis(0).spring_stiffness())
        self.assertEqual(0.0, joint.axis(0).spring_reference())
        self.assertEqual(0.0, joint.axis(0).friction())


if __name__ == "__main__":
    unittest.main()
