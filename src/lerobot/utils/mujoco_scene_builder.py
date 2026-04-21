# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Compose a bimanual YAM MuJoCo scene from the single-arm assets in the i2rt submodule.

The i2rt `combine_arm_and_gripper_xml` helper produces one arm+gripper MJCF; this
module glues two of them into one scene with `left_` / `right_` name prefixes so
MuJoCo's global-name rule is satisfied, and positions the two bases at configurable
offsets. The result is returned as an XML string suitable for
``mujoco.MjModel.from_xml_string``.
"""

from __future__ import annotations

import copy
import xml.etree.ElementTree as ET

# Tags whose @name attribute names a global entity (must be prefixed to avoid collisions
# between left and right copies). We intentionally do NOT include asset tags here because
# the two copies share identical assets and we dedupe them at merge time.
_NAMED_WORLDBODY_TAGS: tuple[str, ...] = ("body", "joint", "site", "geom", "camera", "light")

# Attributes in equality/contact/etc. sections that reference entity names and must be
# prefixed to match the prefixed worldbody names.
_NAME_REF_ATTRS: tuple[str, ...] = (
    "body1",
    "body2",
    "joint1",
    "joint2",
    "site1",
    "site2",
    "geom1",
    "geom2",
    "bodysite",
)


def _prefix_names_in_subtree(elem: ET.Element, prefix: str) -> None:
    """Walk elem recursively; prefix @name on worldbody entity tags in place."""
    for node in elem.iter():
        if node.tag in _NAMED_WORLDBODY_TAGS and node.get("name") is not None:
            node.set("name", prefix + node.get("name"))


def _prefix_name_refs_in_section(elem: ET.Element, prefix: str) -> None:
    """Prefix all name-reference attributes (body1, joint1, etc.) on every child of elem."""
    for node in elem.iter():
        for attr in _NAME_REF_ATTRS:
            val = node.get(attr)
            if val is not None:
                node.set(attr, prefix + val)


def _clone_root(path: str) -> ET.Element:
    """Parse a single-arm MJCF file and return a deep-copied root element."""
    tree = ET.parse(path)
    return copy.deepcopy(tree.getroot())


def _merge_assets(dest: ET.Element, src: ET.Element) -> None:
    """Append assets from src's <asset> into dest's <asset>, skipping duplicates by (tag, name)."""
    src_asset = src.find("asset")
    if src_asset is None:
        return
    dest_asset = dest.find("asset")
    if dest_asset is None:
        dest_asset = ET.SubElement(dest, "asset")
    seen = {(c.tag, c.get("name")) for c in dest_asset}
    for child in src_asset:
        key = (child.tag, child.get("name"))
        if key not in seen:
            dest_asset.append(copy.deepcopy(child))
            seen.add(key)


def _wrap_worldbody_under_root(
    arm_root: ET.Element, wrapper_name: str, pos: tuple[float, float, float]
) -> ET.Element:
    """Move all children of arm_root.worldbody under a new <body name=wrapper_name pos=...> and return that body."""
    worldbody = arm_root.find("worldbody")
    if worldbody is None:
        raise ValueError("arm XML has no <worldbody> element")

    wrapper = ET.Element(
        "body",
        {"name": wrapper_name, "pos": " ".join(f"{v:.6f}" for v in pos)},
    )
    for child in list(worldbody):
        worldbody.remove(child)
        wrapper.append(child)
    return wrapper


def build_bimanual_yam_scene(
    gripper_type: str = "linear_4310",
    left_base_offset: tuple[float, float, float] = (0.0, 0.25, 0.0),
    right_base_offset: tuple[float, float, float] = (0.0, -0.25, 0.0),
) -> str:
    """Compose a two-arm YAM MJCF scene string (arms + grippers, with name prefixes).

    Args:
        gripper_type: name of the gripper to attach to each arm (e.g. "linear_4310").
        left_base_offset: world-frame (x, y, z) of the left arm's base.
        right_base_offset: world-frame (x, y, z) of the right arm's base.

    Returns:
        XML string suitable for ``mujoco.MjModel.from_xml_string``.
    """
    from i2rt.robots.utils import ArmType, GripperType, combine_arm_and_gripper_xml

    arm_type = ArmType.YAM
    if isinstance(gripper_type, str):
        gtype = GripperType.from_string_name(gripper_type)
    else:
        gtype = gripper_type

    # Call combine_arm_and_gripper_xml twice. Each call writes a fresh temp file so the
    # two trees are independent.
    left_path = combine_arm_and_gripper_xml(arm_type, gtype)
    right_path = combine_arm_and_gripper_xml(arm_type, gtype)

    left_root = _clone_root(left_path)
    right_root = _clone_root(right_path)

    # Apply name prefixes to worldbody entities and reference attributes in equality/contact.
    _prefix_names_in_subtree(left_root.find("worldbody"), "left_")
    _prefix_names_in_subtree(right_root.find("worldbody"), "right_")
    for section in ("equality", "contact", "tendon", "sensor"):
        left_section = left_root.find(section)
        if left_section is not None:
            _prefix_name_refs_in_section(left_section, "left_")
        right_section = right_root.find(section)
        if right_section is not None:
            _prefix_name_refs_in_section(right_section, "right_")

    # Build the merged scene. Use left's compiler/defaults/assets as the baseline.
    merged = ET.Element("mujoco", {"model": "bi_yam"})

    # Copy compiler and defaults from the left side (they're identical to right's).
    for tag in ("compiler", "default", "option", "size", "statistic", "visual"):
        elem = left_root.find(tag)
        if elem is not None:
            merged.append(copy.deepcopy(elem))

    # Merge assets: left first (dedupes against itself), then right adds only non-duplicates.
    _merge_assets(merged, left_root)
    _merge_assets(merged, right_root)

    # Build the world: wrap each arm's worldbody content under a named root body.
    worldbody = ET.SubElement(merged, "worldbody")
    # A simple floor + ambient light so the viewer isn't black on first launch.
    ET.SubElement(
        worldbody,
        "light",
        {"directional": "true", "pos": "0 0 3", "dir": "0 0 -1", "castshadow": "false"},
    )
    ET.SubElement(
        worldbody,
        "geom",
        {"name": "floor", "type": "plane", "size": "2 2 0.1", "rgba": "0.9 0.9 0.9 1"},
    )

    left_wrapper = _wrap_worldbody_under_root(left_root, "left_root", left_base_offset)
    right_wrapper = _wrap_worldbody_under_root(right_root, "right_root", right_base_offset)
    worldbody.append(left_wrapper)
    worldbody.append(right_wrapper)

    # Merge top-level equality/contact/tendon/sensor sections from both sides.
    for section in ("equality", "contact", "tendon", "sensor"):
        left_section = left_root.find(section)
        right_section = right_root.find(section)
        if left_section is None and right_section is None:
            continue
        merged_section = ET.SubElement(merged, section)
        if left_section is not None:
            for child in left_section:
                merged_section.append(copy.deepcopy(child))
        if right_section is not None:
            for child in right_section:
                merged_section.append(copy.deepcopy(child))

    return ET.tostring(merged, encoding="unicode")
