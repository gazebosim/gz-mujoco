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

from ignition.math import Pose3d, Quaterniond, Vector3d

from mjcf_to_sdformat.converters.link import add_mjcf_geom_to_sdf
import sdformat_mjcf_utils.sdf_utils as su

import sdformat as sdf


def add_mjcf_worldbody_to_sdf(mjcf_model, world):
    """
    Convert a MJCF worldbody to a SDFormat world

    :param mjcf.Element mjcf_model: The MJCF model
    :param sdf.World world: SDF World to add the models
    """
    model = sdf.Model()
    if mjcf_model.model is not None:
        model.set_name(mjcf_model.model)
    else:
        model.set_name("model")

    link = add_mjcf_geom_to_sdf(mjcf_model.worldbody)
    model.add_link(link)

    body = mjcf_model.worldbody.body

    def iterate_bodies(input_body, model):
        for body in input_body:
            link = add_mjcf_geom_to_sdf(body)
            model.add_link(link)
            iterate_bodies(body.body, model)
    iterate_bodies(body, model)

    world.add_model(model)
