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

from ignition.math import Pose3d, Vector3d, Quaterniond

from mjcf_to_sdformat.converters.link import add_mjcf_link_to_sdf
from mjcf_to_sdformat.converters.light import add_mjcf_light_to_sdf

import sdformat_mjcf_utils.sdf_utils as su

import sdformat as sdf

NUMBER_OF_SDF_MODEL = 0

def add_mjcf_model_to_sdf(mjcf_model, world):
    global NUMBER_OF_SDF_MODEL
    for geom in mjcf_model.worldbody.geom:
        model = sdf.Model()
        model.set_name("model_" + str(NUMBER_OF_SDF_MODEL))
        link = add_mjcf_link_to_sdf(geom, None)
        model.set_static(True)
        model.add_link(link)
        world.add_model(model)
        NUMBER_OF_SDF_MODEL = NUMBER_OF_SDF_MODEL + 1

    for light in mjcf_model.worldbody.light:
        print(light)
        light_sdf = add_mjcf_light_to_sdf(light)
        world.add_light(light_sdf)

    for body in mjcf_model.worldbody.body:
        model = sdf.Model()
        model_pose = [0, 0, 0]
        model_euler = [0, 0, 0]
        if body.pos is not None:
            model_pose = body.pos
        if body.euler is not None:
            model_euler = body.euler
        model.set_raw_pose(Pose3d(su.list_to_vec3d(model_pose),
                           Quaterniond(su.list_to_vec3d(model_euler))))

        for geom in body.geom:
            model.set_name("model_" + str(NUMBER_OF_SDF_MODEL))
            link = add_mjcf_link_to_sdf(geom, body.inertial)
            model.add_link(link)
            NUMBER_OF_SDF_MODEL = NUMBER_OF_SDF_MODEL + 1
        world.add_model(model)
