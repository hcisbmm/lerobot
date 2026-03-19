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

"""Unit tests for effort/torque (.eff) feature support using MockRobotWithEffort."""

import pytest

from tests.mocks.mock_robot_with_effort import MockRobotWithEffort, MockRobotWithEffortConfig


@pytest.fixture
def robot():
    config = MockRobotWithEffortConfig(n_motors=3)
    r = MockRobotWithEffort(config)
    r.connect()
    yield r
    r.disconnect()


def test_observation_features_include_eff(robot):
    """observation_features should always include .eff keys."""
    features = robot.observation_features
    eff_keys = [k for k in features if k.endswith(".eff")]
    pos_keys = [k for k in features if k.endswith(".pos")]
    assert len(eff_keys) == len(pos_keys)
    assert len(eff_keys) == 3
    for key in eff_keys:
        assert features[key] is float


def test_action_features_no_eff(robot):
    """action_features should only contain .pos keys, never .eff."""
    features = robot.action_features
    eff_keys = [k for k in features if k.endswith(".eff")]
    assert len(eff_keys) == 0
    pos_keys = [k for k in features if k.endswith(".pos")]
    assert len(pos_keys) == 3


def test_get_observation_contains_eff(robot):
    """get_observation() should always include .eff values."""
    obs = robot.get_observation()
    eff_keys = [k for k in obs if k.endswith(".eff")]
    pos_keys = [k for k in obs if k.endswith(".pos")]
    assert len(eff_keys) == len(pos_keys)
    assert len(eff_keys) == 3
    for key in eff_keys:
        assert isinstance(obs[key], float)
