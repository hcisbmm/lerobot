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

import pytest

from lerobot.tactile.utils import make_tactile_sensors_from_configs


def test_empty_configs_returns_empty_dict():
    assert make_tactile_sensors_from_configs({}) == {}


def test_unknown_type_raises():
    """Backends are dispatched by `cfg.type`. An unregistered name should be reported clearly."""
    from dataclasses import dataclass

    from lerobot.tactile.configs import TactileSensorConfig

    @TactileSensorConfig.register_subclass("unsupported_for_factory_test")
    @dataclass
    class _Unsupported(TactileSensorConfig):
        pass

    with pytest.raises(ValueError, match="unsupported_for_factory_test"):
        make_tactile_sensors_from_configs({"x": _Unsupported()})
