#!/usr/bin/env python

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

"""Tests for src/lerobot/utils/mujoco_scene_builder.py.

These tests depend on i2rt + mujoco being installed (both come from the `yam` extra);
skip cleanly if either is missing so they don't block environments without the extra.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET

import pytest

pytest.importorskip("i2rt", reason="i2rt required (install lerobot[yam])")
pytest.importorskip("mujoco", reason="mujoco required (install lerobot[yam])")

from lerobot.utils.mujoco_scene_builder import build_bimanual_yam_scene


@pytest.fixture(scope="module")
def scene_xml() -> str:
    return build_bimanual_yam_scene(gripper_type="linear_4310")


@pytest.fixture(scope="module")
def scene_root(scene_xml) -> ET.Element:
    return ET.fromstring(scene_xml)


def test_scene_is_well_formed_xml(scene_xml):
    # Parse must succeed and root must be <mujoco>
    root = ET.fromstring(scene_xml)
    assert root.tag == "mujoco"
    assert root.get("model") == "bi_yam"


def test_scene_compiles_in_mujoco(scene_xml):
    import mujoco

    model = mujoco.MjModel.from_xml_string(scene_xml)
    # Two arms × (6 hinge + 2 slide) = 16 joints
    assert model.njnt == 16
    assert model.nq == 16


def test_all_named_worldbody_entities_are_prefixed(scene_root):
    # Every body/joint/site/camera under worldbody must have a name starting with left_ or right_
    # EXCEPT a small allowlist we add ourselves (the floor and world).
    allowlist = {"floor", "left_root", "right_root"}
    for node in scene_root.find("worldbody").iter():
        if node.tag not in ("body", "joint", "site", "camera"):
            continue
        name = node.get("name")
        if name is None or name in allowlist:
            continue
        assert name.startswith("left_") or name.startswith("right_"), (
            f"Entity {node.tag} name={name!r} is not prefixed"
        )


def test_joint_names_match_canonical_layout(scene_xml):
    import mujoco

    model = mujoco.MjModel.from_xml_string(scene_xml)
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(model.njnt)]
    expected_per_arm = [f"joint{i}" for i in range(1, 9)]  # joint1..joint8 (6 arm + 2 finger)
    for side, offset in (("left", 0), ("right", 8)):
        for j, jname in enumerate(expected_per_arm):
            assert names[offset + j] == f"{side}_{jname}", names


def test_grasp_sites_exist_per_side(scene_xml):
    import mujoco

    model = mujoco.MjModel.from_xml_string(scene_xml)
    site_names = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, i) for i in range(model.nsite)}
    assert "left_grasp_site" in site_names
    assert "right_grasp_site" in site_names


def test_no_duplicate_names_in_worldbody(scene_root):
    seen: dict[str, int] = {}
    for node in scene_root.find("worldbody").iter():
        if node.tag not in ("body", "joint", "site"):
            continue
        name = node.get("name")
        if name is None:
            continue
        seen[name] = seen.get(name, 0) + 1
    dupes = {n: c for n, c in seen.items() if c > 1}
    assert not dupes, f"duplicate names found: {dupes}"


def test_base_offsets_applied(scene_root):
    # left_root and right_root wrapper bodies exist with the expected pos attributes.
    left = scene_root.find(".//body[@name='left_root']")
    right = scene_root.find(".//body[@name='right_root']")
    assert left is not None and right is not None
    # Default offsets: left y=+0.25, right y=-0.25
    assert "0.250000" in left.get("pos") or "0.25" in left.get("pos")
    assert "-0.250000" in right.get("pos") or "-0.25" in right.get("pos")


def test_custom_base_offsets():
    import mujoco

    xml = build_bimanual_yam_scene(
        gripper_type="linear_4310",
        left_base_offset=(0.0, 0.5, 0.1),
        right_base_offset=(0.0, -0.5, 0.1),
    )
    root = ET.fromstring(xml)
    left_pos = root.find(".//body[@name='left_root']").get("pos").split()
    right_pos = root.find(".//body[@name='right_root']").get("pos").split()
    assert [float(p) for p in left_pos] == pytest.approx([0.0, 0.5, 0.1])
    assert [float(p) for p in right_pos] == pytest.approx([0.0, -0.5, 0.1])
    # Compiles
    mujoco.MjModel.from_xml_string(xml)


def test_equality_constraints_preserved_with_prefixed_joints(scene_root):
    # linear_4310 has an equality coupling joint7 ↔ joint8 per side. After prefixing,
    # we should see left_joint7 ↔ left_joint8 and right_joint7 ↔ right_joint8.
    equality = scene_root.find("equality")
    assert equality is not None, "equality section missing"
    pairs = [
        (e.get("joint1"), e.get("joint2"))
        for e in equality.findall("joint")
        if e.get("joint1") and e.get("joint2")
    ]
    assert ("left_joint7", "left_joint8") in pairs
    assert ("right_joint7", "right_joint8") in pairs
