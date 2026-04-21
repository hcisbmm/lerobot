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

import importlib
import sys
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip("rerun", reason="rerun-sdk is required (install lerobot[viz])")

from lerobot.types import TransitionKey
from lerobot.utils.constants import OBS_STATE


@pytest.fixture
def mock_rerun(monkeypatch):
    """
    Provide a mock `rerun` module so tests don't depend on the real library.
    Also reload the module-under-test so it binds to this mock `rr`.
    """
    calls = []

    class DummyScalar:
        def __init__(self, value):
            self.value = float(value)

    class DummyImage:
        def __init__(self, arr):
            self.arr = arr

    def dummy_log(key, obj=None, **kwargs):
        # Accept either positional `obj` or keyword `entity` and record remaining kwargs.
        if obj is None and "entity" in kwargs:
            obj = kwargs.pop("entity")
        calls.append((key, obj, kwargs))

    dummy_rr = SimpleNamespace(
        __name__="rerun",
        __package__="rerun",
        __spec__=SimpleNamespace(name="rerun", submodule_search_locations=None),
        Scalars=DummyScalar,
        Image=DummyImage,
        log=dummy_log,
        init=lambda *a, **k: None,
        spawn=lambda *a, **k: None,
    )

    # Inject fake module into sys.modules
    monkeypatch.setitem(sys.modules, "rerun", dummy_rr)

    # Now import and reload the module under test, to bind to our rerun mock
    import lerobot.utils.visualization_utils as vu

    importlib.reload(vu)

    # Expose both the reloaded module and the call recorder
    yield vu, calls


def _keys(calls):
    """Helper to extract just the keys logged to rr.log"""
    return [k for (k, _obj, _kw) in calls]


def _obj_for(calls, key):
    """Find the first object logged under a given key."""
    for k, obj, _kw in calls:
        if k == key:
            return obj
    raise KeyError(f"Key {key} not found in calls: {calls}")


def _kwargs_for(calls, key):
    for k, _obj, kw in calls:
        if k == key:
            return kw
    raise KeyError(f"Key {key} not found in calls: {calls}")


def test_log_rerun_data_envtransition_scalars_and_image(mock_rerun):
    vu, calls = mock_rerun

    # Build EnvTransition dict
    obs = {
        f"{OBS_STATE}.temperature": np.float32(25.0),
        # CHW image should be converted to HWC for rr.Image
        "observation.camera": np.zeros((3, 10, 20), dtype=np.uint8),
    }
    act = {
        "action.throttle": 0.7,
        # 1D array should log individual Scalars with suffix _i
        "action.vector": np.array([1.0, 2.0], dtype=np.float32),
    }
    transition = {
        TransitionKey.OBSERVATION: obs,
        TransitionKey.ACTION: act,
    }

    # Extract observation and action data from transition like in the real call sites
    obs_data = transition.get(TransitionKey.OBSERVATION, {})
    action_data = transition.get(TransitionKey.ACTION, {})
    vu.log_rerun_data(observation=obs_data, action=action_data)

    # We expect:
    # - observation.state.temperature -> Scalars
    # - observation.camera -> Image (HWC) with static=True
    # - action.throttle -> Scalars
    # - action.vector_0, action.vector_1 -> Scalars
    expected_keys = {
        f"{OBS_STATE}.temperature",
        "observation.camera",
        "action.throttle",
        "action.vector_0",
        "action.vector_1",
    }
    assert set(_keys(calls)) == expected_keys

    # Check scalar types and values
    temp_obj = _obj_for(calls, f"{OBS_STATE}.temperature")
    assert type(temp_obj).__name__ == "DummyScalar"
    assert temp_obj.value == pytest.approx(25.0)

    throttle_obj = _obj_for(calls, "action.throttle")
    assert type(throttle_obj).__name__ == "DummyScalar"
    assert throttle_obj.value == pytest.approx(0.7)

    v0 = _obj_for(calls, "action.vector_0")
    v1 = _obj_for(calls, "action.vector_1")
    assert type(v0).__name__ == "DummyScalar"
    assert type(v1).__name__ == "DummyScalar"
    assert v0.value == pytest.approx(1.0)
    assert v1.value == pytest.approx(2.0)

    # Check image handling: CHW -> HWC
    img_obj = _obj_for(calls, "observation.camera")
    assert type(img_obj).__name__ == "DummyImage"
    assert img_obj.arr.shape == (10, 20, 3)  # transposed
    assert _kwargs_for(calls, "observation.camera").get("static", False) is True  # static=True for images


def test_log_rerun_data_plain_list_ordering_and_prefixes(mock_rerun):
    vu, calls = mock_rerun

    # First dict without prefixes treated as observation
    # Second dict without prefixes treated as action
    obs_plain = {
        "temp": 1.5,
        # Already HWC image => should stay as-is
        "img": np.zeros((5, 6, 3), dtype=np.uint8),
        "none": None,  # should be skipped
    }
    act_plain = {
        "throttle": 0.3,
        "vec": np.array([9, 8, 7], dtype=np.float32),
    }

    # Extract observation and action data from list like the old function logic did
    # First dict was treated as observation, second as action
    vu.log_rerun_data(observation=obs_plain, action=act_plain)

    # Expected keys with auto-prefixes
    expected = {
        "observation.temp",
        "observation.img",
        "action.throttle",
        "action.vec_0",
        "action.vec_1",
        "action.vec_2",
    }
    logged = set(_keys(calls))
    assert logged == expected

    # Scalars
    t = _obj_for(calls, "observation.temp")
    assert type(t).__name__ == "DummyScalar"
    assert t.value == pytest.approx(1.5)

    throttle = _obj_for(calls, "action.throttle")
    assert type(throttle).__name__ == "DummyScalar"
    assert throttle.value == pytest.approx(0.3)

    # Image stays HWC
    img = _obj_for(calls, "observation.img")
    assert type(img).__name__ == "DummyImage"
    assert img.arr.shape == (5, 6, 3)
    assert _kwargs_for(calls, "observation.img").get("static", False) is True

    # Vectors
    for i, val in enumerate([9, 8, 7]):
        o = _obj_for(calls, f"action.vec_{i}")
        assert type(o).__name__ == "DummyScalar"
        assert o.value == pytest.approx(val)


def test_log_rerun_data_kwargs_only(mock_rerun):
    vu, calls = mock_rerun

    vu.log_rerun_data(
        observation={"observation.temp": 10.0, "observation.gray": np.zeros((8, 8, 1), dtype=np.uint8)},
        action={"action.a": 1.0},
    )

    keys = set(_keys(calls))
    assert "observation.temp" in keys
    assert "observation.gray" in keys
    assert "action.a" in keys

    temp = _obj_for(calls, "observation.temp")
    assert type(temp).__name__ == "DummyScalar"
    assert temp.value == pytest.approx(10.0)

    img = _obj_for(calls, "observation.gray")
    assert type(img).__name__ == "DummyImage"
    assert img.arr.shape == (8, 8, 1)  # remains HWC
    assert _kwargs_for(calls, "observation.gray").get("static", False) is True

    a = _obj_for(calls, "action.a")
    assert type(a).__name__ == "DummyScalar"
    assert a.value == pytest.approx(1.0)


# ---------------------------------------------------------------------------
# Shared obs-parsing helpers (used by TorqueVisualizer + MujocoTorqueVisualizer)
# ---------------------------------------------------------------------------


def test_parse_effort_key_joints():
    from lerobot.utils.visualization_utils import parse_effort_key

    assert parse_effort_key("left_joint_0.eff") == ("left", "joint_0")
    assert parse_effort_key("left_joint_5.eff") == ("left", "joint_5")
    assert parse_effort_key("right_joint_0.eff") == ("right", "joint_0")
    assert parse_effort_key("right_joint_5.eff") == ("right", "joint_5")


def test_parse_effort_key_grippers():
    from lerobot.utils.visualization_utils import parse_effort_key

    assert parse_effort_key("left_gripper.eff") == ("left", "gripper")
    assert parse_effort_key("right_gripper.eff") == ("right", "gripper")


def test_parse_effort_key_accepts_pos_suffix():
    from lerobot.utils.visualization_utils import parse_effort_key

    assert parse_effort_key("left_joint_2.pos") == ("left", "joint_2")
    assert parse_effort_key("right_gripper.pos") == ("right", "gripper")


def test_parse_effort_key_rejects_malformed():
    from lerobot.utils.visualization_utils import parse_effort_key

    assert parse_effort_key("no_dot_at_all") is None
    assert parse_effort_key("unknown_arm_joint_0.eff") is None
    assert parse_effort_key("left_gripperextra.eff") is None
    assert parse_effort_key("left_joint_x.eff") is None
    assert parse_effort_key(".eff") is None


def test_is_gripper_key():
    from lerobot.utils.visualization_utils import is_gripper_key

    assert is_gripper_key("left_gripper.eff") is True
    assert is_gripper_key("right_gripper.pos") is True
    assert is_gripper_key("left_joint_0.eff") is False
    assert is_gripper_key("right_joint_5.eff") is False


def test_extract_ordered_eff_zero_fills_missing():
    from lerobot.utils.visualization_utils import BI_YAM_JOINT_ORDER, extract_ordered

    obs = {
        "left_joint_0.eff": 1.5,
        "right_gripper.eff": -0.3,
    }
    arr = extract_ordered(obs, "eff")
    assert arr.shape == (14,)
    assert arr.dtype == np.float64
    assert arr[BI_YAM_JOINT_ORDER.index("left_joint_0")] == pytest.approx(1.5)
    assert arr[BI_YAM_JOINT_ORDER.index("right_gripper")] == pytest.approx(-0.3)
    assert arr[BI_YAM_JOINT_ORDER.index("left_joint_1")] == 0.0


def test_extract_ordered_pos_same_interface():
    from lerobot.utils.visualization_utils import extract_ordered

    obs = {f"left_joint_{i}.pos": float(i) for i in range(6)}
    obs["left_gripper.pos"] = 0.05
    arr = extract_ordered(obs, "pos")
    assert arr[0:6].tolist() == [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
    assert arr[6] == pytest.approx(0.05)
    assert (arr[7:] == 0.0).all()


def test_torque_visualizer_uses_shared_helpers():
    from lerobot.utils.visualization_utils import TorqueVisualizer

    obs = {
        "left_joint_0.eff": 0.5,
        "left_gripper.eff": 0.1,
        "right_joint_3.eff": -0.2,
        "right_gripper.eff": 0.3,
        "left_joint_0.pos": 1.0,  # non-eff, must be filtered out
    }

    viz_full = TorqueVisualizer(ee_only=False)
    eff_full = viz_full._filter_eff_keys(obs)
    assert set(eff_full.keys()) == {
        "left_joint_0.eff", "left_gripper.eff", "right_joint_3.eff", "right_gripper.eff"
    }

    viz_ee = TorqueVisualizer(ee_only=True)
    eff_ee = viz_ee._filter_eff_keys(obs)
    assert set(eff_ee.keys()) == {"left_gripper.eff", "right_gripper.eff"}
