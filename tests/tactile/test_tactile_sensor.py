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

import abc

import pytest

from lerobot.tactile.tactile_sensor import TactileSensor


def test_tactile_sensor_is_abstract():
    assert issubclass(TactileSensor, abc.ABC)
    with pytest.raises(TypeError):
        TactileSensor()  # type: ignore[abstract]


def test_required_abstract_methods():
    required = {"connect", "disconnect", "is_connected", "shape", "dtype", "async_read"}
    assert required.issubset(TactileSensor.__abstractmethods__)


def test_default_latest_timestamp_returns_none():
    """Subclasses get a sensible default for latest_timestamp without re-implementing it."""
    from dataclasses import dataclass

    from lerobot.tactile.configs import TactileSensorConfig

    @TactileSensorConfig.register_subclass("dummy_for_latest_ts_test")
    @dataclass
    class _DummyConfig(TactileSensorConfig):
        pass

    class _Stub(TactileSensor):
        def connect(self) -> None: ...
        def disconnect(self) -> None: ...
        @property
        def is_connected(self) -> bool: return False
        @property
        def shape(self): return (48,)
        @property
        def dtype(self): return None  # placeholder
        def async_read(self): return None  # placeholder

    s = _Stub(_DummyConfig())
    assert s.latest_timestamp is None
