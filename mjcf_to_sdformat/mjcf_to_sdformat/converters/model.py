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

from sdformat_mjcf.converters.link import add_mjcf_geom_to_sdf

import sdformat as sdf

NUMBER_OF_SDF_MODEL = 0


def add_mjcf_model_to_sdf(mjcf_model, world):
    global NUMBER_OF_SDF_MODEL
    for geom in mjcf_model.worldbody.geom:
        model = sdf.Model()
        model.set_name("model_" + str(NUMBER_OF_SDF_MODEL))
        link = add_mjcf_geom_to_sdf(geom)
        model.set_static(True)
        model.add_link(link)
        world.add_model(model)
        NUMBER_OF_SDF_MODEL = NUMBER_OF_SDF_MODEL + 1
    for body in mjcf_model.worldbody.body:
        for geom in body.geom:
            model = sdf.Model()
            model.set_name("model_" + str(NUMBER_OF_SDF_MODEL))
            link = add_mjcf_geom_to_sdf(geom)
            model.add_link(link)
            world.add_model(model)
            NUMBER_OF_SDF_MODEL = NUMBER_OF_SDF_MODEL + 1
