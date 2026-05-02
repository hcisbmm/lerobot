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

"""Pure parsing helpers for XELA Server v1.7.x WebSocket frames.

Wire format (XELA Software Manual v1.7.6, p. 37):

    {
      "message": <int seq>,
      "time": <unix_ts>,
      "sensors": <int K>,
      "<sensor_id>": {
        "data": "FA00,FB00,FC00,...",   # comma-separated 4-char hex (uint16), 3 per taxel
        "sensor": "<sensor_id>",
        "taxels": <int>,
        "model": "<model_string>",
        "calibrated": null | [floats]
      },
      ...
    }

The first message is a welcome payload: ``{"message": "Welcome", ...}`` and must be skipped.
"""

import json
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray  # type: ignore  # TODO: add type stubs for numpy.typing

AXES_PER_TAXEL = 3


@dataclass(frozen=True)
class ParseResult:
    seq: int
    timestamp: float
    raw: NDArray[np.float32]
    calibrated: NDArray[np.float32] | None


def parse_hex_csv(s: str) -> NDArray[np.float32]:
    """Parse a comma-separated string of 4-char hex codes into a float32 array.

    Empty string raises ValueError. Each token is parsed via ``int(tok, 16)``.
    """
    if not s:
        raise ValueError("parse_hex_csv: empty input")
    return np.array([int(tok, 16) for tok in s.split(",")], dtype=np.float32)


def is_welcome(payload: dict) -> bool:
    """True if this is the connect-time welcome message that should be skipped."""
    return payload.get("message") == "Welcome"


def parse_frame(raw_json: str, *, sensor_id: str, expected_model: str) -> ParseResult:
    """Parse a single WebSocket frame string into a ParseResult.

    Raises:
        KeyError: if `sensor_id` is not present in the frame.
        ValueError: on model mismatch, taxel-count mismatch, or empty data.
    """
    payload = json.loads(raw_json)
    if sensor_id not in payload:
        raise KeyError(
            f"sensor_id {sensor_id!r} not in frame; available top-level keys: "
            f"{sorted(k for k in payload if k.isdigit())}"
        )
    sensor = payload[sensor_id]

    if sensor["model"] != expected_model:
        raise ValueError(
            f"model mismatch: frame reports {sensor['model']!r} but config expected "
            f"{expected_model!r}. Re-run xela_conf or fix the config."
        )

    raw = parse_hex_csv(sensor["data"])
    expected_len = int(sensor["taxels"]) * AXES_PER_TAXEL
    if raw.size != expected_len:
        raise ValueError(
            f"taxel count mismatch: frame data has {raw.size} values but "
            f"{sensor['taxels']} taxels × {AXES_PER_TAXEL} axes = {expected_len} expected."
        )

    cal_field = sensor.get("calibrated")
    calibrated: NDArray[np.float32] | None = (
        None if cal_field is None else np.asarray(cal_field, dtype=np.float32)
    )

    return ParseResult(
        seq=int(payload["message"]),
        timestamp=float(payload["time"]),
        raw=raw,
        calibrated=calibrated,
    )
