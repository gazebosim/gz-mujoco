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

from sdformat_mjcf.sdformat_to_mjcf.converters.geometry import (
    add_collision,
    add_visual,
)
from sdformat_mjcf.sdformat_to_mjcf.converters.light import add_light
from sdformat_mjcf.sdformat_to_mjcf.converters.material import add_material
from sdformat_mjcf.sdformat_to_mjcf.converters.sensor import add_sensor
import sdformat_mjcf.utils.sdf_utils as su


def add_link(body, link, parent_name="world", link_pose=None):
    """
    Converts a link from SDFormat to MJCF and add it to the given
    body/worldbody.

    :param mjcf.Element body: The MJCF body to which the body is added.
    :param sdformat.Link link: The SDFormat link to be converted.
    :param str parent_name: Name of parent link.
    :param ignition.math.Pose3d link_pose: Pose of link. This is optional and
    if set to None, the pose will be resolved from the link.
    :return: The newly created MJCF body.
    :rtype: mjcf.Element
    """
    # Currently the following SDFormat elements inside <link> are ignored
    # during conversion
    #     - self_collide (TODO (azeey))
    #     - kinematic
    #     - must_be_base_link
    #     - velocity_decay
    #     - projector
    #     - audio_sink
    #     - audio_source
    #     - battery
    #     - particle_emitter
    if link_pose is not None:
        pose = link_pose
    else:
        sem_pose = link.semantic_pose()
        if parent_name == 'world':
            pose = su.graph_resolver.resolve_pose(sem_pose)
        else:
            pose = su.graph_resolver.resolve_pose(sem_pose, parent_name)

    body = body.add("body",
                    name=su.find_unique_name(body, "body", link.name()),
                    pos=su.vec3d_to_list(pose.pos()),
                    euler=su.quat_to_euler_list(pose.rot()))

    # SDFormat allows specifying diagonal and off-diagonal terms of the inertia
    # matrix in addition to setting the orientation of the inertia frame.
    # Mujoco, on the other hand seems to allow one of the two: Either set the
    # orientation and provide just the digonal terms, or set the `fullinertia`
    # attribute and not set the orientation. We choose the latter here.
    mass = link.inertial().mass_matrix().mass()
    moi = link.inertial().moi()
    if mass > 0:
        fullinertia = [
            moi(0, 0),
            moi(1, 1),
            moi(2, 2),
            moi(0, 1),
            moi(0, 2),
            moi(1, 2)
        ]
        body.add("inertial",
                 mass=mass,
                 pos=su.vec3d_to_list(link.inertial().pose().pos()),
                 fullinertia=fullinertia)
    else:
        raise RuntimeError(
            "Mass of link {} should not be 0".format(link.name()))
    for ci in range(link.collision_count()):
        col = link.collision_by_index(ci)
        if col.geometry() is not None:
            add_collision(body, col)

    for vi in range(link.visual_count()):
        vis = link.visual_by_index(vi)
        if vis.geometry() is not None:
            visual_geom = add_visual(body, vis)
            if vis.material() is not None:
                add_material(visual_geom, vis.material())

    for li in range(link.light_count()):
        light = link.light_by_index(li)
        if light is not None:
            add_light(body, light)

    for si in range(link.sensor_count()):
        sensor = link.sensor_by_index(si)
        if sensor is not None:
            add_sensor(body, sensor)

    return body
