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

from sdformat_mjcf.converters.geometry import (add_mjcf_visual_to_sdf,
    add_mjcf_collision_to_sdf)
import sdformat_mjcf.sdf_utils as su

import sdformat as sdf

NUMBER_OF_SDF_LINK = 0


def add_mjcf_link_to_sdf(geom):
    global NUMBER_OF_SDF_LINK
    link = sdf.Link()
    link.set_name(geom.type + "_" + str(NUMBER_OF_SDF_LINK))

    if geom.group is None:
        visual = add_mjcf_visual_to_sdf(geom)
        if visual is not None:
            link.add_visual(visual)

        col = add_mjcf_collision_to_sdf(geom)
        if col is not None:
            link.add_collision(col)


    NUMBER_OF_SDF_LINK = NUMBER_OF_SDF_LINK + 1
    return link
