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

import json

import numpy as np
import pytest

from lerobot.tactile.xela.parser import (
    ParseResult,
    is_welcome,
    parse_frame,
    parse_hex_csv,
)


def _xr1944_data_string(values_uint16: list[int]) -> str:
    """Build the comma-separated hex CSV string used in the XELA wire format."""
    assert len(values_uint16) == 48
    return ",".join(f"{v:04X}" for v in values_uint16)


def test_parse_hex_csv_basic():
    s = "FA00,FB00,FC00,0001"
    out = parse_hex_csv(s)
    np.testing.assert_array_equal(out, np.array([0xFA00, 0xFB00, 0xFC00, 0x0001], dtype=np.float32))
    assert out.dtype == np.float32


def test_parse_hex_csv_empty_raises():
    with pytest.raises(ValueError):
        parse_hex_csv("")


def test_is_welcome_message():
    assert is_welcome({"message": "Welcome"})
    assert not is_welcome({"message": 786, "time": 1.0})


def test_parse_frame_full():
    raw_values = [(i * 137 + 7) & 0xFFFF for i in range(48)]
    msg = {
        "message": 786,
        "time": 1619157045.6195483,
        "sensors": 1,
        "1": {
            "data": _xr1944_data_string(raw_values),
            "sensor": "1",
            "taxels": 16,
            "model": "XR1944",
            "calibrated": None,
        },
    }
    result = parse_frame(json.dumps(msg), sensor_id="1", expected_model="XR1944")
    assert isinstance(result, ParseResult)
    assert result.seq == 786
    assert result.timestamp == pytest.approx(1619157045.6195483)
    assert result.raw.shape == (48,)
    assert result.raw.dtype == np.float32
    np.testing.assert_array_equal(result.raw, np.asarray(raw_values, dtype=np.float32))
    assert result.calibrated is None


def test_parse_frame_with_calibrated():
    raw_values = [0x1234] * 48
    cal_values = [0.001 * i for i in range(48)]
    msg = {
        "message": 1,
        "time": 1.0,
        "sensors": 1,
        "1": {
            "data": _xr1944_data_string(raw_values),
            "sensor": "1",
            "taxels": 16,
            "model": "XR1944",
            "calibrated": cal_values,
        },
    }
    result = parse_frame(json.dumps(msg), sensor_id="1", expected_model="XR1944")
    assert result.calibrated is not None
    assert result.calibrated.shape == (48,)
    assert result.calibrated.dtype == np.float32
    np.testing.assert_allclose(result.calibrated, np.asarray(cal_values, dtype=np.float32))


def test_parse_frame_model_mismatch_raises():
    msg = {
        "message": 1,
        "time": 1.0,
        "sensors": 1,
        "1": {
            "data": _xr1944_data_string([0] * 48),
            "sensor": "1",
            "taxels": 16,
            "model": "XR1944",
            "calibrated": None,
        },
    }
    with pytest.raises(ValueError, match="model mismatch"):
        parse_frame(json.dumps(msg), sensor_id="1", expected_model="XR2244")


def test_parse_frame_missing_sensor_id_raises():
    msg = {
        "message": 1,
        "time": 1.0,
        "sensors": 1,
        "1": {
            "data": _xr1944_data_string([0] * 48),
            "sensor": "1",
            "taxels": 16,
            "model": "XR1944",
            "calibrated": None,
        },
    }
    with pytest.raises(KeyError, match="sensor_id"):
        parse_frame(json.dumps(msg), sensor_id="2", expected_model="XR1944")


def test_parse_frame_taxel_count_mismatch_raises():
    """If `data` length disagrees with `taxels` field, refuse."""
    bad_data = ",".join(f"{0:04X}" for _ in range(45))  # 45 instead of 48
    msg = {
        "message": 1,
        "time": 1.0,
        "sensors": 1,
        "1": {"data": bad_data, "sensor": "1", "taxels": 16, "model": "XR1944", "calibrated": None},
    }
    with pytest.raises(ValueError, match="taxel count"):
        parse_frame(json.dumps(msg), sensor_id="1", expected_model="XR1944")
