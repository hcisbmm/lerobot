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


def test_graceful_close_logs_info_not_warning(patched_ws_app, caplog):
    """When disconnect() triggered the close, _on_close must use INFO, not WARNING."""
    import logging

    sensor = XelaTactileSensor(XelaTactileConfig(receive_timeout_s=0.5))
    sensor.connect()
    fake = _wait_for_first(patched_ws_app)
    fake.run_forever_called.wait(timeout=2.0)

    with caplog.at_level(logging.INFO, logger="lerobot.tactile.xela.xela_tactile"):
        sensor.disconnect()  # sets _stop, then closes WS — graceful

    close_records = [r for r in caplog.records if "WS closed" in r.message]
    assert close_records, "expected at least one WS-closed log entry"
    # All graceful-close records must be INFO, never WARNING.
    for rec in close_records:
        assert rec.levelno == logging.INFO, (
            f"graceful disconnect logged at {rec.levelname}: {rec.message!r}"
        )


def test_unexpected_close_logs_warning(patched_ws_app, caplog):
    """When the server side closes (not us), _on_close must use WARNING."""
    import logging

    sensor = XelaTactileSensor(
        XelaTactileConfig(receive_timeout_s=0.5, reconnect_backoff_s=10.0)  # don't reconnect
    )
    sensor.connect()
    fake = _wait_for_first(patched_ws_app)
    fake.run_forever_called.wait(timeout=2.0)

    with caplog.at_level(logging.INFO, logger="lerobot.tactile.xela.xela_tactile"):
        # Simulate a server-initiated close: drive run_forever to return without setting _stop.
        fake.close_called = True
        # The reader thread observes the close and calls _on_close. Wait briefly.
        time.sleep(0.05)

    sensor.disconnect()

    unexpected_records = [
        r for r in caplog.records
        if "WS closed unexpectedly" in r.message
    ]
    assert unexpected_records, "expected at least one unexpected-close warning"
    for rec in unexpected_records:
        assert rec.levelno == logging.WARNING


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


def test_seq_reset_on_reconnect(patched_ws_app):
    """After server restart with seq counter reset, new frames must be accepted.

    Regression: `_last_seq` accumulating across reconnects caused every
    post-restart frame (whose `xela_server` seq starts back near 0) to be
    silently dropped as "out-of-order", freezing async_read() at the last
    pre-disconnect frame indefinitely.
    """
    sensor = XelaTactileSensor(
        XelaTactileConfig(receive_timeout_s=2.0, reconnect_backoff_s=0.05)
    )
    sensor.connect()
    fake1 = _wait_for_first(patched_ws_app)
    fake1.run_forever_called.wait(timeout=2.0)
    fake1.on_message(fake1, _xr1944_msg(seq=10000, value=0x0010))
    np.testing.assert_array_equal(
        sensor.async_read(), np.full(48, 0x10, dtype=np.float32)
    )

    # Simulate server restart: close the current WS, wait for the reader to
    # spawn a new WebSocketApp, then deliver a low-seq frame on the new one.
    fake1.close_called = True
    deadline = time.time() + 3.0
    while len(patched_ws_app) < 2 and time.time() < deadline:
        time.sleep(0.01)
    assert len(patched_ws_app) >= 2, "reader did not reconnect"
    fake2 = patched_ws_app[1]
    fake2.run_forever_called.wait(timeout=2.0)
    fake2.on_message(fake2, _xr1944_msg(seq=0, value=0x0042))

    # The new frame must be accepted, NOT dropped as out-of-order.
    np.testing.assert_array_equal(
        sensor.async_read(), np.full(48, 0x42, dtype=np.float32)
    )
    sensor.disconnect()


def test_use_calibrated_logs_no_op_warning(caplog):
    """v1 documents `use_calibrated=True` as a no-op; warn at construct time."""
    import logging

    with caplog.at_level(logging.WARNING, logger="lerobot.tactile.xela.xela_tactile"):
        XelaTactileSensor(XelaTactileConfig(use_calibrated=True))
    no_op_records = [r for r in caplog.records if "no effect in v1" in r.message]
    assert no_op_records, "expected a one-shot 'no effect in v1' warning"
    assert no_op_records[0].levelno == logging.WARNING


def test_use_calibrated_default_silent(caplog):
    """Default `use_calibrated=False` must not emit the no-op warning."""
    import logging

    with caplog.at_level(logging.WARNING, logger="lerobot.tactile.xela.xela_tactile"):
        XelaTactileSensor(XelaTactileConfig())  # default: use_calibrated=False
    no_op_records = [r for r in caplog.records if "no effect in v1" in r.message]
    assert not no_op_records, "default config must not log the no-op warning"


def test_stale_frame_warning_after_idle(patched_ws_app, caplog):
    """async_read() logs a WARNING when the last frame is older than 1 s."""
    import logging

    sensor = XelaTactileSensor(
        XelaTactileConfig(receive_timeout_s=2.0, reconnect_backoff_s=10.0)
    )
    sensor.connect()
    fake = _wait_for_first(patched_ws_app)
    fake.run_forever_called.wait(timeout=2.0)
    # ts=1.0 from epoch — ancient by definition; staleness check fires
    fake.on_message(fake, _xr1944_msg(seq=1, value=0x0007, ts=1.0))

    with caplog.at_level(logging.WARNING, logger="lerobot.tactile.xela.xela_tactile"):
        _ = sensor.async_read()

    stale_records = [r for r in caplog.records if "stale" in r.message]
    assert stale_records, "expected at least one staleness warning"
    sensor.disconnect()
