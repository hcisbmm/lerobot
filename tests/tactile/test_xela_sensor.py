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
import threading
import time
from unittest.mock import patch

import numpy as np
import pytest

websocket = pytest.importorskip("websocket")  # skip on machines without websocket-client

from lerobot.tactile.xela.configuration_xela import XelaTactileConfig
from lerobot.tactile.xela.xela_tactile import XelaTactileSensor


def _xr1944_msg(seq: int, value: int = 0x1234, ts: float = 1.0) -> str:
    sensor_block = {
        "data": ",".join(f"{value:04X}" for _ in range(48)),
        "sensor": "1",
        "taxels": 16,
        "model": "XR1944",
        "calibrated": None,
    }
    return json.dumps({"message": seq, "time": ts, "sensors": 1, "1": sensor_block})


class _FakeWSApp:
    """Stand-in for `websocket.WebSocketApp` whose handlers we drive by hand."""

    def __init__(self, url, *, on_open=None, on_message=None, on_close=None, on_error=None, **_):
        self.url = url
        self.on_open = on_open
        self.on_message = on_message
        self.on_close = on_close
        self.on_error = on_error
        self.close_called = False
        self.run_forever_called = threading.Event()

    def run_forever(self, **_):
        self.run_forever_called.set()
        if self.on_open:
            self.on_open(self)
        # Block until close() is called so the reader thread treats us like a real WS.
        while not self.close_called:
            time.sleep(0.001)
        if self.on_close:
            self.on_close(self, 1000, "")

    def close(self):
        self.close_called = True


@pytest.fixture
def patched_ws_app():
    holder: list[_FakeWSApp] = []

    def factory(url, **kwargs):
        app = _FakeWSApp(url, **kwargs)
        holder.append(app)
        return app

    with patch("lerobot.tactile.xela.xela_tactile.websocket.WebSocketApp", side_effect=factory):
        yield holder


def _wait_for_first(holder, timeout: float = 2.0) -> _FakeWSApp:
    """Poll the WebSocketApp holder until the daemon thread has constructed one.

    The reader thread first calls `_resolve_host` (a brief socket round-trip when
    `host="auto"`) and only then constructs the WebSocketApp. Without this poll,
    test-thread access to `holder[0]` can race that construction.
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        if holder:
            return holder[0]
        time.sleep(0.005)
    raise AssertionError(f"WebSocketApp not constructed within {timeout:.2f}s")


def test_connect_and_disconnect_lifecycle(patched_ws_app):
    sensor = XelaTactileSensor(XelaTactileConfig(receive_timeout_s=0.5))
    assert sensor.is_connected is False
    sensor.connect()
    # Wait until run_forever has actually been entered (worker thread is up).
    assert _wait_for_first(patched_ws_app).run_forever_called.wait(timeout=2.0)
    assert sensor.is_connected is True
    sensor.disconnect()
    assert sensor.is_connected is False


def test_async_read_returns_first_frame(patched_ws_app):
    sensor = XelaTactileSensor(XelaTactileConfig(receive_timeout_s=2.0))
    sensor.connect()
    fake = _wait_for_first(patched_ws_app)
    fake.run_forever_called.wait(timeout=2.0)

    # Drive Welcome (must be ignored) then a real frame.
    fake.on_message(fake, json.dumps({"message": "Welcome"}))
    fake.on_message(fake, _xr1944_msg(seq=1, value=0x00FF))

    frame = sensor.async_read()
    assert frame.shape == (48,)
    assert frame.dtype == np.float32
    np.testing.assert_array_equal(frame, np.full(48, 255.0, dtype=np.float32))
    assert sensor.latest_timestamp == pytest.approx(1.0)
    sensor.disconnect()


def test_out_of_order_frames_are_dropped(patched_ws_app):
    sensor = XelaTactileSensor(XelaTactileConfig(receive_timeout_s=2.0))
    sensor.connect()
    fake = _wait_for_first(patched_ws_app)
    fake.run_forever_called.wait(timeout=2.0)

    fake.on_message(fake, _xr1944_msg(seq=10, value=0x0010))
    fake.on_message(fake, _xr1944_msg(seq=5, value=0x0005))   # older — drop
    fake.on_message(fake, _xr1944_msg(seq=11, value=0x0011))

    frame = sensor.async_read()
    np.testing.assert_array_equal(frame, np.full(48, 0x11, dtype=np.float32))
    sensor.disconnect()


def test_disconnect_returns_last_good_frame(patched_ws_app):
    """After WS close, `async_read` must keep returning the last successfully parsed frame."""
    sensor = XelaTactileSensor(XelaTactileConfig(receive_timeout_s=2.0,
                                                 reconnect_backoff_s=10.0))  # never reconnect during test
    sensor.connect()
    fake = _wait_for_first(patched_ws_app)
    fake.run_forever_called.wait(timeout=2.0)
    fake.on_message(fake, _xr1944_msg(seq=1, value=0x0042))
    first = sensor.async_read()

    # Simulate disconnect — drive on_close.
    fake.close_called = True
    time.sleep(0.05)  # let the reader thread observe the close.

    second = sensor.async_read()  # must not raise; must return last-good
    np.testing.assert_array_equal(first, second)
    sensor.disconnect()


def test_async_read_before_any_frame_raises(patched_ws_app):
    sensor = XelaTactileSensor(XelaTactileConfig(receive_timeout_s=0.1))
    sensor.connect()
    _wait_for_first(patched_ws_app).run_forever_called.wait(timeout=2.0)
    with pytest.raises(TimeoutError):
        sensor.async_read()
    sensor.disconnect()


def test_async_read_before_connect_raises(patched_ws_app):
    sensor = XelaTactileSensor(XelaTactileConfig())
    with pytest.raises(RuntimeError, match="not connected"):
        sensor.async_read()


def test_shape_and_dtype(patched_ws_app):
    sensor = XelaTactileSensor(XelaTactileConfig())
    assert sensor.shape == (48,)
    assert sensor.dtype == np.float32


def test_resolve_host_passthrough():
    from lerobot.tactile.xela.xela_tactile import _resolve_host

    assert _resolve_host("127.0.0.1") == "127.0.0.1"
    assert _resolve_host("192.168.1.79") == "192.168.1.79"
    assert _resolve_host("xela.lan") == "xela.lan"


def test_resolve_host_auto_returns_an_ipv4():
    from lerobot.tactile.xela.xela_tactile import _resolve_host

    out = _resolve_host("auto")
    parts = out.split(".")
    assert len(parts) == 4
    for p in parts:
        n = int(p)
        assert 0 <= n <= 255
