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

import random
from dataclasses import dataclass
from functools import cached_property

from lerobot.robots import RobotConfig
from lerobot.utils.decorators import check_if_not_connected
from tests.mocks.mock_robot import MockRobot, MockRobotConfig


@RobotConfig.register_subclass("mock_robot_with_effort")
@dataclass
class MockRobotWithEffortConfig(MockRobotConfig):
    pass


class MockRobotWithEffort(MockRobot):
    """Mock Robot that includes .eff (effort/torque) keys in observations and features."""

    config_class = MockRobotWithEffortConfig
    name = "mock_robot_with_effort"

    @property
    def _effort_ft(self) -> dict[str, type]:
        return {f"{motor}.eff": float for motor in self.motors}

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._effort_ft, **self._cameras_ft}

    @check_if_not_connected
    def get_observation(self):
        obs = super().get_observation()
        for motor in self.motors:
            obs[f"{motor}.eff"] = random.uniform(-10, 10)
        return obs
