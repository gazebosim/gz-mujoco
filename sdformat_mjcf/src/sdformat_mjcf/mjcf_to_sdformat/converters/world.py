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

from ignition.math import Vector3d

from sdformat_mjcf.mjcf_to_sdformat.converters.joint import (mjcf_joint_to_sdf,
                                                             add_fixed_joint)
from sdformat_mjcf.mjcf_to_sdformat.converters.light import mjcf_light_to_sdf
from sdformat_mjcf.mjcf_to_sdformat.converters.link import mjcf_body_to_sdf
from sdformat_mjcf.mjcf_to_sdformat.converters.sensor import (
    mjcf_camera_sensor_to_sdf, mjcf_accelerometer_gyro_sensor_to_sdf,
    mjcf_force_torque_sensor_to_sdf)

import sdformat_mjcf.utils.sdf_utils as su
from sdformat_mjcf.utils.defaults import MjcfModifiers

import sdformat as sdf


def mjcf_worldbody_to_sdf(mjcf_root, physics, world,
                          export_world_plugins=False):
    """
    Convert a MJCF worldbody to a SDFormat world

    :param mjcf.RootElement mjcf_root: The MJCF root element
    :param mujoco.Physics physics: Mujoco Physics
    :param sdf.World world: SDF World to add the models
    :param boolean export_world_plugins: If this this flag is True then
    system plugin will be exported (SceneBroadcaster, UserCommands, Physics,
    Sensors), otherwise no system plugin will be exported.
    """
    model = sdf.Model()
    if mjcf_root.model is not None:
        model.set_name(mjcf_root.model)
    else:
        model.set_name("model")
    model.set_self_collide(True)

    modifiers = MjcfModifiers(mjcf_root)

    model_static = sdf.Model()

    for light in mjcf_root.worldbody.light:
        modifiers.apply_modifiers_to_element(light)
        light_sdf = mjcf_light_to_sdf(light)
        world.add_light(light_sdf)

    link = mjcf_body_to_sdf(mjcf_root.worldbody, physics, modifiers=modifiers)
    model_static.set_name(su.find_unique_name(
        mjcf_root.worldbody, "geom", "static"))

    for camera in mjcf_root.worldbody.camera:
        sensor = mjcf_camera_sensor_to_sdf(camera)
        if sensor is not None:
            link.add_sensor(sensor)

    model_static.add_link(link)
    model_static.set_static(True)
    world.add_model(model_static)

    body = mjcf_root.worldbody.body

    if mjcf_root.option is not None:
        if mjcf_root.option.flag.gravity == "disable":
            world.set_gravity(Vector3d(0, 0, 0))
        elif mjcf_root.option.gravity is not None:
            world.set_gravity(su.list_to_vec3d(mjcf_root.option.gravity))
        if mjcf_root.option.magnetic is not None:
            world.set_magnetic_field(
                su.list_to_vec3d(mjcf_root.option.magnetic))
        if mjcf_root.option.wind is not None:
            world.set_wind_linear_velocity(
                su.list_to_vec3d(mjcf_root.option.wind))

    def iterate_bodies(input_body,
                       model,
                       body_parent_name=None):
        for body in input_body:
            body_name = body.name

            link = mjcf_body_to_sdf(body,
                                    physics,
                                    body_parent_name=body_parent_name,
                                    modifiers=modifiers)
            for camera in body.camera:
                sensor = mjcf_camera_sensor_to_sdf(camera)
                if sensor is not None:
                    link.add_sensor(sensor)

            serial_joints_created = []
            serial_sdf_joints_created = []
            serial_link_created = [link]
            for joint in body.joint:
                if modifiers is not None:
                    modifiers.apply_modifiers_to_element(joint)

                def check_child_name(model, child_name):
                    """
                    Check if there is a joint in the model with this child name
                    Return True if the child name exists, False otherwise
                    :param sdf.Model model: Model to check the joints
                    :param str child_name: Name of the child joint to check
                    :return: True if the child name exists, False otherwise
                    :rtype bool:
                    """
                    for j in range(model.joint_count()):
                        joint_sdf = model.joint_by_index(j)
                        if (joint_sdf.child_link_name() == child_name):
                            return True
                    return False

                def serial_joint_check(model, parent_name, child_name):
                    """
                    Check if a joint requires to add a serial joint.
                    :param sdf.Model model: Model to check the joint
                    :param str parent_name: Name of the child joint to check
                    :param str child_name: Name of the child joint to check
                    """
                    child_name_result = child_name
                    for j in range(model.joint_count()):
                        joint_sdf = model.joint_by_index(j)
                        # if there is a joint with this parent and child names
                        # we should add a new link and attach the new joint to
                        # this new link
                        if joint_sdf.parent_link_name() == parent_name and \
                           joint_sdf.child_link_name() == child_name:
                            index = 0
                            while True:
                                # we need to create a new name for the child
                                # For this, we will check all the child names
                                # and we will add a suffix if the name already
                                # exists
                                new_child_name = child_name + "_" + str(index)
                                if not check_child_name(model, new_child_name):
                                    # Create a new link and add it to the model
                                    dummy_link = sdf.Link()
                                    dummy_link.set_name(new_child_name)
                                    dummy_link.set_raw_pose(link.raw_pose())
                                    dummy_link.set_pose_relative_to(
                                        link.pose_relative_to())
                                    serial_link_created.append(dummy_link)
                                    child_name_result = new_child_name
                                    model.add_joint(joint_sdf)
                                    return child_name_result
                                index = index + 1
                    return child_name_result

                # We can have more than two link in serie, we should check if
                # the names are already in use in any joint.
                child_name_result = body.name
                parent_name_found = body_parent_name
                while True:
                    result = serial_joint_check(
                        model, parent_name_found, child_name_result)
                    if child_name_result not in serial_joints_created:
                        child_name_result = result
                        break
                    parent_name_found = child_name_result
                    child_name_result = result

                # create a joint
                if child_name_result == body_name:
                    joint_sdf = mjcf_joint_to_sdf(joint,
                                                  body_parent_name,
                                                  body_name)
                    serial_sdf_joints_created.append(joint_sdf)
                # create a serial joint
                else:
                    joint_sdf = mjcf_joint_to_sdf(joint,
                                                  body_name,
                                                  child_name_result)
                    serial_joints_created.append(child_name_result)
                    serial_sdf_joints_created.append(joint_sdf)
                    body_name = child_name_result

                if joint_sdf is not None:
                    model.add_joint(joint_sdf)
            if len(body.joint) == 0 and body.freejoint is None:
                joint_sdf = add_fixed_joint(body_parent_name, body.name)
                model.add_joint(joint_sdf)

            # We might have more than one link, if there is the case,
            # we should remove visuals and colllisions in all the previous
            # link only if the joint is in the same place
            if len(serial_link_created) > 1:
                if serial_sdf_joints_created[0] is not None and \
                   serial_sdf_joints_created[-1] is not None:
                    if serial_sdf_joints_created[0].raw_pose() == \
                       serial_sdf_joints_created[-1].raw_pose():
                        serial_link_created[-1].add_visual(
                            link.visual_by_index(0))
                        serial_link_created[-1].add_collision(
                            link.collision_by_index(0))
                        serial_link_created[-1].set_inertial(link.inertial())
                        link.clear_visuals()
                        link.clear_collisions()
                        link.inertial().mass_matrix().set_mass(0)

            # Add all links to the world
            for i in range(len(serial_link_created)):
                model.add_link(serial_link_created[i])

            model.add_link(link)
            iterate_bodies(body.body,
                           model,
                           body_name)
    iterate_bodies(body, model)

    if mjcf_root.sensor is not None:
        for accel in mjcf_root.sensor.accelerometer:
            if accel is not None:
                mjcf_accelerometer_gyro_sensor_to_sdf(accel, model)
        for gyro in mjcf_root.sensor.gyro:
            if gyro is not None:
                mjcf_accelerometer_gyro_sensor_to_sdf(gyro, model)
        for force in mjcf_root.sensor.force:
            if force is not None:
                mjcf_force_torque_sensor_to_sdf(force, model)
        for torque in mjcf_root.sensor.torque:
            if torque is not None:
                mjcf_force_torque_sensor_to_sdf(torque, model)

    if export_world_plugins:
        plugins = {
            "ignition-gazebo-physics-system":
                "ignition::gazebo::systems::Physics",
            "ignition-gazebo-sensors-system":
                "ignition::gazebo::systems::Sensors",
            "ignition-gazebo-user-commands-system":
                "ignition::gazebo::systems::UserCommands",
            "ignition-gazebo-scene-broadcaster-system":
                "ignition::gazebo::systems::SceneBroadcaster"
        }
        for [key, value] in plugins.items():
            plugin = sdf.Plugin(key, value)
            world.add_plugin(plugin)

    # A non-static SDFormat model is required to have at least one link, so if
    # we haven't added any links to the model, we should not add it to the
    # world.
    if model.link_count() > 0:
        world.add_model(model)
