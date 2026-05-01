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

import numpy as np
import pytest

from lerobot.tactile.mock.mock_tactile import MockTactileConfig, MockTactileSensor
from lerobot.tactile.utils import make_tactile_sensors_from_configs


def test_mock_config_type_string():
    assert MockTactileConfig().type == "mock"


def test_mock_sensor_lifecycle():
    sensor = MockTactileSensor(MockTactileConfig())
    assert sensor.is_connected is False
    sensor.connect()
    assert sensor.is_connected is True
    sensor.disconnect()
    assert sensor.is_connected is False


def test_mock_sensor_async_read_shape_dtype():
    sensor = MockTactileSensor(MockTactileConfig(num_taxels=16, axes=3))
    sensor.connect()
    frame = sensor.async_read()
    assert frame.shape == (48,)
    assert frame.dtype == np.float32
    sensor.disconnect()


def test_mock_sensor_async_read_before_connect_raises():
    sensor = MockTactileSensor(MockTactileConfig())
    with pytest.raises(RuntimeError, match="not connected"):
        sensor.async_read()


def test_mock_sensor_async_read_is_deterministic_per_seed():
    """Two sensors with the same seed must produce the same first frame."""
    a = MockTactileSensor(MockTactileConfig(seed=42))
    b = MockTactileSensor(MockTactileConfig(seed=42))
    a.connect()
    b.connect()
    np.testing.assert_array_equal(a.async_read(), b.async_read())
    a.disconnect()
    b.disconnect()


def test_factory_dispatches_to_mock_backend():
    sensors = make_tactile_sensors_from_configs({"left": MockTactileConfig()})
    assert isinstance(sensors["left"], MockTactileSensor)


def test_mock_sensor_provides_calibrated_default_false():
    sensor = MockTactileSensor(MockTactileConfig())
    assert sensor.provides_calibrated is False
    with pytest.raises(NotImplementedError):
        sensor.async_read_calibrated()


def test_mock_sensor_async_read_calibrated_when_enabled():
    sensor = MockTactileSensor(MockTactileConfig(provides_calibrated=True))
    sensor.connect()
    assert sensor.provides_calibrated is True
    cal = sensor.async_read_calibrated()
    assert cal.shape == (48,)
    assert cal.dtype == np.float32
    # Sanity: cosine waveform scaled to ~0.001 — distinguishable from sine raw.
    assert np.abs(cal).max() <= 0.0011
    sensor.disconnect()


def test_mock_sensor_latest_timestamp_after_read():
    sensor = MockTactileSensor(MockTactileConfig())
    sensor.connect()
    assert sensor.latest_timestamp is None
    sensor.async_read()
    assert isinstance(sensor.latest_timestamp, float)
    sensor.disconnect()
