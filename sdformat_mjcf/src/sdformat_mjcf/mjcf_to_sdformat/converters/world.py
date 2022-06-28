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

from ignition.math import Vector3d, MassMatrix3d, Inertiald

from sdformat_mjcf.mjcf_to_sdformat.converters.joint import (mjcf_joint_to_sdf,
                                                             add_fixed_joint)
from sdformat_mjcf.mjcf_to_sdformat.converters.link import mjcf_body_to_sdf
from sdformat_mjcf.mjcf_to_sdformat.converters.sensor import (
    mjcf_camera_sensor_to_sdf, mjcf_accelerometer_gyro_sensor_to_sdf,
    mjcf_force_torque_sensor_to_sdf)

import sdformat_mjcf.utils.sdf_utils as su
from sdformat_mjcf.utils.defaults import MjcfModifiers

import sdformat as sdf


def _is_floating_body(body):
    freejoint = body.get_children("freejoint")
    if freejoint:
        return True
    all_child_joints = body.get_children("joint")
    for joint in all_child_joints:
        if joint.type == "free":
            return True
    return False


def _add_body_to_model(body, physics, model, modifiers):
    try:
        parent_name = body.parent.name
    except AttributeError:
        parent_name = "world"

    link = mjcf_body_to_sdf(body,
                            physics,
                            body_parent_name=parent_name,
                            modifiers=modifiers)
    for camera in body.camera:
        sensor = mjcf_camera_sensor_to_sdf(camera)
        if sensor is not None:
            link.add_sensor(sensor)

    serial_sdf_joints_created = []
    serial_link_created = [link]
    # Some physics engines may not like 0 mass, so we create a small mass and
    # set the inertia from a unit sphere. The radius is arbitrary as long as
    # the inertia values are unlikely to affect the dynamics.
    small_mass_matrix = MassMatrix3d()
    small_mass_matrix.set_from_sphere(1e-6, 1)
    small_inertial = Inertiald()
    small_inertial.set_mass_matrix(small_mass_matrix)

    for joint_num, joint in enumerate(body.joint):
        if modifiers is not None:
            modifiers.apply_modifiers_to_element(joint)

        if joint_num < len(body.joint) - 1:
            child_name = su.find_unique_name(body, "body",
                                             f"{body.name}_dummy_{joint_num}")
            joint_sdf = mjcf_joint_to_sdf(joint, parent_name, child_name)
            serial_sdf_joints_created.append(joint_sdf)
            link = sdf.Link()
            link.set_name(child_name)
            link.set_pose_relative_to(body.name)
            link.set_inertial(small_inertial)
            serial_link_created.append(link)
            parent_name = child_name
        else:
            joint_sdf = mjcf_joint_to_sdf(joint, parent_name, body.name)
            serial_sdf_joints_created.append(joint_sdf)

        if joint_sdf is not None:
            model.add_joint(joint_sdf)

    if len(body.joint) == 0 and not _is_floating_body(body):
        joint_sdf = add_fixed_joint(parent_name, body.name)
        model.add_joint(joint_sdf)

    # Add all links to the model
    for i in range(len(serial_link_created)):
        model.add_link(serial_link_created[i])

    for child_body in body.body:
        _add_body_to_model(child_body, physics, model, modifiers)

    return link


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
    modifiers = MjcfModifiers(mjcf_root)

    for mesh_asset in mjcf_root.asset.mesh:
        modifiers.apply_modifiers_to_element(mesh_asset)

    for material_asset in mjcf_root.asset.material:
        modifiers.apply_modifiers_to_element(material_asset)

    model_static = sdf.Model()

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

    models = []
    root_body_to_model = {}
    for body in mjcf_root.worldbody.body:
        model = sdf.Model()
        link = _add_body_to_model(body, physics, model, modifiers)
        model.set_name(f"model_for_{link.name()}")
        models.append(model)
        root_body_to_model[body] = model

    # Walk up the hierarchy until we find body in root_body_to_model since
    # we only store top level bodies (kinematic roots) in root_body_to_model
    def _get_root_body(body):
        while body.parent != body.root.worldbody:
            body = body.parent
        return body

    def _get_model_from_sensor(sensor):
        if hasattr(sensor, "site"):
            return root_body_to_model[_get_root_body(sensor.site.parent)]

        raise RuntimeError(
            f"Sensor {sensor} not attached to a site or body")

    if mjcf_root.sensor is not None:
        for accel in mjcf_root.sensor.accelerometer:
            if accel is not None:
                model = _get_model_from_sensor(accel)
                mjcf_accelerometer_gyro_sensor_to_sdf(accel, model)
        for gyro in mjcf_root.sensor.gyro:
            if gyro is not None:
                model = _get_model_from_sensor(gyro)
                mjcf_accelerometer_gyro_sensor_to_sdf(gyro, model)
        for force in mjcf_root.sensor.force:
            if force is not None:
                model = _get_model_from_sensor(force)
                mjcf_force_torque_sensor_to_sdf(force, model)
        for torque in mjcf_root.sensor.torque:
            if torque is not None:
                model = _get_model_from_sensor(torque)
                mjcf_force_torque_sensor_to_sdf(torque, model)

    if export_world_plugins:
        plugins = {
            "ignition-gazebo-physics-system":
                "gz::sim::systems::Physics",
            "ignition-gazebo-sensors-system":
                "gz::sim::systems::Sensors",
            "ignition-gazebo-user-commands-system":
                "gz::sim::systems::UserCommands",
            "ignition-gazebo-scene-broadcaster-system":
                "gz::sim::systems::SceneBroadcaster",
            "ignition-gazebo-forcetorque-system":
                "gz::sim::systems::ForceTorque",
            "ignition-gazebo-altimeter-system":
                "gz::sim::systems::Altimeter",
            "ignition-gazebo-imu-system":
                "gz::sim::systems::Imu"
        }
        for [key, value] in plugins.items():
            plugin = sdf.Plugin(key, value)
            world.add_plugin(plugin)

    # A non-static SDFormat model is required to have at least one link, so if
    # we haven't added any links to the model, we should not add it to the
    # world.
    for model in models:
        if model is not None and model.link_count() > 0:
            world.add_model(model)
