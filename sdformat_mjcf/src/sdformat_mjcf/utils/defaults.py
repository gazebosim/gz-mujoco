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

from dm_control import mjcf

# Based on https://github.com/deepmind/mujoco/blob/d7ae7f54338a8ea02c34f4a969722182481eb72b/unity/Runtime/Importer/MjXmlModifiers.cs  # noqa


class MjcfModifiers:
    """Process default classes and their inheritance and apply the resulting
    default class to a given element."""

    def __init__(self, root):
        """
        Initialize the class with the root element. Note that this class
        mutates the default element (root.default) and its descendants in order
        to processes the inheritance hierarchy.
        :param mjcf.RootElement root: The MJCF root element
        """
        self.root = root
        self._apply_inherited_default(self.root.default, None)

    def apply_modifiers_to_element(self, elem):
        """
        Given an element, e.g., geom, body, etc, this copies into the element
        the attributes of the default class associated with the element.
        :param mjcf.Element elem: The MJCF element to process.
        """
        dclass = self._get_default_class(elem)
        def_elem = dclass.get_children(elem.tag)
        self._copy_attributes(def_elem, elem)

    def _get_default_class(self, elem):
        if hasattr(elem, 'dclass') and elem.dclass is not None:
            return elem.dclass

        cur_elem = elem.parent
        while cur_elem is not None:
            if hasattr(cur_elem,
                       "childclass") and cur_elem.childclass is not None:
                return cur_elem.childclass
            else:
                cur_elem = cur_elem.parent

        # No childclass applies, so use the main default class
        return elem.root.default

    def _apply_inherited_default(self, child_def, parent_def):
        if parent_def is not None:
            for elem in child_def.all_children():
                if elem.tag != "default":
                    parent_elem = parent_def.get_children(elem.tag)
                    self._copy_attributes(parent_elem, elem)
        for grand_child_def in child_def.default:
            self._apply_inherited_default(grand_child_def, child_def)

    def _copy_attributes(self, from_elem, to_elem):
        to_attrs = to_elem.get_attributes().keys()
        for attr, val in from_elem.get_attributes().items():
            if attr not in to_attrs:
                to_elem.set_attributes(**{attr: val})


# Applies the defaults on all geoms and prints the resulting xml. Only used for
# degugging purposes.
if __name__ == "__main__":
    import sys
    model = mjcf.from_path(sys.argv[1])
    modifiers = MjcfModifiers(model)
    for geom in model.find_all("geom"):
        modifiers.apply_modifiers_to_element(geom)
    model.default.remove()
    print(model.to_xml_string())
