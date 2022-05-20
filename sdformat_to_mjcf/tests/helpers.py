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

"""Test helpers"""

import pathlib
import unittest
from contextlib import contextmanager

from sdformat_mjcf_utils import sdf_utils as su

TEST_RESOURCES_DIR = pathlib.Path(__file__).resolve().parent / 'resources'


class TestGraphResolverImpl(su.GraphResolverImplBase):

    def resolve_pose(self, sem_pose, relative_to=None):
        try:
            return super().resolve_pose(sem_pose, relative_to)
        except RuntimeError:
            return sem_pose.raw_pose()

    def resolve_axis_xyz(self, joint_axis):
        try:
            return super().resolve_axis_xyz(joint_axis)
        except RuntimeError:
            return joint_axis.xyz()

    def resolve_parent_link_name(self, joint):
        try:
            return super().resolve_parent_link_name(joint)
        except RuntimeError:
            return joint.parent_link_name()

    def resolve_child_link_name(self, joint):
        try:
            return super().resolve_child_link_name(joint)
        except RuntimeError:
            return joint.child_link_name()


def setup_test_graph_resolver():
    su.graph_resolver.resolver = TestGraphResolverImpl()


def reset_graph_resolver():
    su.graph_resolver = su.GraphResolver()


class TestCase(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        setup_test_graph_resolver()

    @classmethod
    def tearDownClass(cls):
        reset_graph_resolver()


@contextmanager
def test_graph_resolver_context():
    setup_test_graph_resolver()
    yield
    reset_graph_resolver()
