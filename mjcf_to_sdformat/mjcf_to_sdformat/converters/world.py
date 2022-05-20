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

from mjcf_to_sdformat.converters.link import mjcf_geom_to_sdf

import sdformat as sdf


def mjcf_worldbody_to_sdf(mjcf_root, world):
    """
    Convert a MJCF worldbody to a SDFormat world

    :param mjcf.RootElement mjcf_root: The MJCF root element
    :param sdf.World world: SDF World to add the models
    """
    model = sdf.Model()
    if mjcf_root.model is not None:
        model.set_name(mjcf_root.model)
    else:
        model.set_name("model")

    link = mjcf_geom_to_sdf(mjcf_root.worldbody)
    model.add_link(link)

    body = mjcf_root.worldbody.body

    def iterate_bodies(input_body, model):
        for body in input_body:
            link = mjcf_geom_to_sdf(body)
            model.add_link(link)
            iterate_bodies(body.body, model)
    iterate_bodies(body, model)

    world.add_model(model)
