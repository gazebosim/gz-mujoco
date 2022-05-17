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

from ignition.math import Color

import sdformat as sdf


def add_mjcf_material_to_sdf(geom):
    material = sdf.Material()
    if geom.material is not None:
        if  geom.material.rgba is not None:
            material.set_diffuse(Color(geom.material.rgba[0], geom.material.rgba[1], geom.material.rgba[2], geom.material.rgba[3]))
            material.set_ambient(Color(geom.material.rgba[0], geom.material.rgba[1], geom.material.rgba[2], geom.material.rgba[3]))
            material.set_specular(Color(geom.material.rgba[0], geom.material.rgba[1], geom.material.rgba[2], geom.material.rgba[3]))
            material.set_emissive(Color(geom.material.rgba[0], geom.material.rgba[1], geom.material.rgba[2], geom.material.rgba[3]))
            return material
    if geom.rgba is not None:
        material.set_diffuse(Color(geom.rgba[0], geom.rgba[1], geom.rgba[2], geom.rgba[3]))
        material.set_ambient(Color(geom.rgba[0], geom.rgba[1], geom.rgba[2], geom.rgba[3]))
        material.set_specular(Color(geom.rgba[0], geom.rgba[1], geom.rgba[2], geom.rgba[3]))
        material.set_emissive(Color(geom.rgba[0], geom.rgba[1], geom.rgba[2], geom.rgba[3]))
        return material
    return None
