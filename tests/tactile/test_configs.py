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

from dataclasses import dataclass

import draccus

from lerobot.tactile.configs import TactileSensorConfig


def test_base_config_is_choice_registry():
    """TactileSensorConfig must be a draccus.ChoiceRegistry so subclasses can register."""
    assert issubclass(TactileSensorConfig, draccus.ChoiceRegistry)


def test_subclass_registration_and_type_property():
    """A registered subclass must report its registered name via the `type` property."""

    @TactileSensorConfig.register_subclass("dummy_tactile_for_test")
    @dataclass
    class _DummyConfig(TactileSensorConfig):
        x: int = 1

    cfg = _DummyConfig(x=2)
    assert cfg.type == "dummy_tactile_for_test"
    assert cfg.x == 2
