# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
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

from unittest.mock import MagicMock

import numpy as np
import pytest

from lerobot.robots.bi_yam_follower.bi_yam_follower import BiYamFollower
from lerobot.robots.bi_yam_follower.config_bi_yam_follower import BiYamFollowerConfig
from lerobot.tactile.mock.mock_tactile import MockTactileConfig


def test_config_default_no_tactile():
    cfg = BiYamFollowerConfig()
    assert cfg.tactile_sensors == {}


def test_config_accepts_tactile_sensor_dict():
    cfg = BiYamFollowerConfig(tactile_sensors={"right_finger_r": MockTactileConfig()})
    assert "right_finger_r" in cfg.tactile_sensors
    assert cfg.tactile_sensors["right_finger_r"].type == "mock"


@pytest.fixture
def follower_with_mock_tactile():
    """Construct a BiYamFollower with both arm clients fully mocked + a MockTactileSensor."""
    cfg = BiYamFollowerConfig(tactile_sensors={"right_finger_r": MockTactileConfig()})
    follower = BiYamFollower(cfg)

    fake_obs = {
        "joint_pos": np.zeros(6, dtype=np.float32),
        "gripper_pos": np.zeros(1, dtype=np.float32),
        "joint_eff": np.zeros(6, dtype=np.float32),
        "gripper_eff": np.zeros(1, dtype=np.float32),
    }

    for arm_attr in ("left_arm", "right_arm"):
        client = MagicMock()
        client.is_connected = True
        client.num_dofs.return_value = 7
        client.get_observations.return_value = fake_obs
        setattr(follower, arm_attr, client)

    follower._left_dofs = 7
    follower._right_dofs = 7

    # The BiYamFollower.__init__ already constructed self.tactile_sensors via the factory.
    # We just need to bring them online (BiYamFollower.connect() would also try the portal
    # arm-client path; we skip that here and connect tactile sensors directly).
    for s in follower.tactile_sensors.values():
        s.connect()

    yield follower

    for s in follower.tactile_sensors.values():
        s.disconnect()


def test_observation_features_include_tactile(follower_with_mock_tactile):
    feats = follower_with_mock_tactile.observation_features
    assert "observation.tactile.right_finger_r" in feats
    assert feats["observation.tactile.right_finger_r"] == (48,)


def test_get_observation_includes_tactile_vector(follower_with_mock_tactile):
    obs = follower_with_mock_tactile.get_observation(include_cameras=False)
    assert "observation.tactile.right_finger_r" in obs
    arr = obs["observation.tactile.right_finger_r"]
    assert isinstance(arr, np.ndarray)
    assert arr.shape == (48,)
    assert arr.dtype == np.float32


@pytest.fixture
def follower_with_mock_calibrated_tactile():
    """BiYamFollower with a MockTactileSensor that exposes the calibrated path."""
    cfg = BiYamFollowerConfig(tactile_sensors={"right_finger_r": MockTactileConfig(provides_calibrated=True)})
    follower = BiYamFollower(cfg)

    fake_obs = {
        "joint_pos": np.zeros(6, dtype=np.float32),
        "gripper_pos": np.zeros(1, dtype=np.float32),
        "joint_eff": np.zeros(6, dtype=np.float32),
        "gripper_eff": np.zeros(1, dtype=np.float32),
    }
    for arm_attr in ("left_arm", "right_arm"):
        client = MagicMock()
        client.is_connected = True
        client.num_dofs.return_value = 7
        client.get_observations.return_value = fake_obs
        setattr(follower, arm_attr, client)
    follower._left_dofs = 7
    follower._right_dofs = 7

    for s in follower.tactile_sensors.values():
        s.connect()
    yield follower
    for s in follower.tactile_sensors.values():
        s.disconnect()


def test_observation_features_include_cal_when_provided(follower_with_mock_calibrated_tactile):
    """When sensor.provides_calibrated, _tactile_ft must include the .cal sibling key."""
    feats = follower_with_mock_calibrated_tactile.observation_features
    assert "observation.tactile.right_finger_r" in feats
    assert "observation.tactile.right_finger_r.cal" in feats
    assert feats["observation.tactile.right_finger_r"] == (48,)
    assert feats["observation.tactile.right_finger_r.cal"] == (48,)


def test_get_observation_includes_cal_when_provided(follower_with_mock_calibrated_tactile):
    """When sensor.provides_calibrated, get_observation emits the .cal column."""
    obs = follower_with_mock_calibrated_tactile.get_observation(include_cameras=False)
    assert "observation.tactile.right_finger_r" in obs
    assert "observation.tactile.right_finger_r.cal" in obs
    cal = obs["observation.tactile.right_finger_r.cal"]
    assert isinstance(cal, np.ndarray)
    assert cal.shape == (48,)
    assert cal.dtype == np.float32
    # Mock cal is cosine × 0.001 — distinguishable from raw (sine, unscaled).
    assert np.abs(cal).max() <= 0.0011


def test_observation_features_no_cal_when_not_provided(follower_with_mock_tactile):
    """Default MockTactileConfig has provides_calibrated=False — no .cal key."""
    feats = follower_with_mock_tactile.observation_features
    assert "observation.tactile.right_finger_r" in feats
    assert "observation.tactile.right_finger_r.cal" not in feats
