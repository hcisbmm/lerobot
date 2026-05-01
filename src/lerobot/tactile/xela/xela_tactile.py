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
import logging
import socket
import threading
import time
from typing import TYPE_CHECKING

import numpy as np
from numpy.typing import NDArray  # type: ignore  # TODO: add type stubs for numpy.typing

from lerobot.utils.import_utils import _websocket_client_available

from ..tactile_sensor import TactileSensor
from .configuration_xela import XelaTactileConfig
from .parser import is_welcome, parse_frame

if TYPE_CHECKING or _websocket_client_available:
    import websocket  # type: ignore  # provided by websocket-client
else:
    websocket = None  # type: ignore[assignment]

logger = logging.getLogger(__name__)

_MAX_BACKOFF_S = 2.0


def _resolve_host(host: str) -> str:
    """Resolve ``"auto"`` to the host's primary outbound-route IP.

    Mirrors xela_server's heuristic: it binds to the IP the OS would use to
    reach the public internet (typically the LAN IP, e.g. 192.168.x.x).
    Anything else is returned unchanged.

    On an air-gapped host (no default route to a public IP), falls back to
    127.0.0.1 with a one-shot WARNING so the operator can attribute a
    subsequent connect TimeoutError to the *real* cause (host unreachable)
    rather than to xela_server itself.
    """
    if host != "auto":
        return host
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))  # no packet is actually sent
        return s.getsockname()[0]
    except OSError as e:
        logger.warning(
            "XELA host=auto failed to resolve outbound-route IP (%s); "
            "falling back to 127.0.0.1. If xela_server bound to a LAN IP, set "
            "host explicitly in XelaTactileConfig (e.g. host='192.168.1.79').",
            e,
        )
        return "127.0.0.1"
    finally:
        s.close()


class XelaTactileSensor(TactileSensor):
    """Client for XELA Server v1.7.x exposed over WebSocket on ``ws://host:port``.

    A daemon thread holds the connection open, parses every JSON frame, and stores the
    most recent good ``np.ndarray((N,), float32)`` in a single-slot buffer. Consumers
    call :meth:`async_read` to retrieve the latest frame; on transport failures the
    sensor returns the last-good-frame and reconnects in the background.
    """

    def __init__(self, config: XelaTactileConfig):
        super().__init__(config)
        self.config: XelaTactileConfig = config
        self._shape = config.expected_shape  # validate model name early

        self._connected: bool = False
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._frame_event = threading.Event()
        self._latest: NDArray[np.float32] | None = None
        self._latest_cal: NDArray[np.float32] | None = None
        self._latest_ts: float | None = None
        self._last_seq: int = -1
        self._tare_offset: NDArray[np.float32] | None = None

        self._ws_app: "websocket.WebSocketApp | None" = None
        self._thread: threading.Thread | None = None

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def shape(self) -> tuple[int, ...]:
        return self._shape

    @property
    def dtype(self) -> np.dtype:
        return np.dtype(np.float32)

    @property
    def latest_timestamp(self) -> float | None:
        return self._latest_ts

    def connect(self) -> None:
        if not _websocket_client_available:
            raise ImportError(
                "websocket-client is not installed. "
                "Install with `pip install -e '.[yam]'` or `pip install websocket-client`."
            )
        if self._connected:
            return
        self._stop.clear()
        self._frame_event.clear()
        self._thread = threading.Thread(
            target=self._reader_loop, name=f"xela-{self.config.host}:{self.config.port}", daemon=True
        )
        # Set is_connected only AFTER thread.start() succeeds — if spawn fails,
        # is_connected must not lie to BiYamFollower's readiness check.
        self._thread.start()
        self._connected = True

    def disconnect(self) -> None:
        self._connected = False
        self._stop.set()
        if self._ws_app is not None:
            try:
                self._ws_app.close()
            except Exception:
                pass
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self._ws_app = None
        self._thread = None

    def async_read(self) -> NDArray[np.float32]:
        if not self._connected and self._latest is None:
            raise RuntimeError("XelaTactileSensor is not connected; call connect() first.")
        if self._latest is None:
            # Block up to receive_timeout_s for the very first frame.
            if not self._frame_event.wait(timeout=self.config.receive_timeout_s):
                raise TimeoutError(
                    f"No XELA frame received within {self.config.receive_timeout_s:.2f}s. "
                    f"Is xela_server running on {self.config.host}:{self.config.port}?"
                )
        with self._lock:
            assert self._latest is not None
            frame = self._latest.copy()
            ts = self._latest_ts
        if ts is not None and (time.time() - ts) > 1.0:
            logger.warning(
                "XELA tactile frame is stale: now-ts=%.2fs (sensor at %s:%d)",
                time.time() - ts, self.config.host, self.config.port,
            )
        if self._tare_offset is not None:
            frame = frame - self._tare_offset
        return frame

    # ----- internals -----

    def _reader_loop(self) -> None:
        backoff = self.config.reconnect_backoff_s
        resolved_host = _resolve_host(self.config.host)
        if resolved_host != self.config.host:
            logger.info("XELA host=%r resolved to %s", self.config.host, resolved_host)
        url = f"ws://{resolved_host}:{self.config.port}"
        while not self._stop.is_set():
            # Reset the sequence-monotonicity guard per connection: a fresh
            # `xela_server` (operator-restarted mid-session, or recovered after a
            # crash) emits seq=0,1,2,… which would all be <= the old _last_seq
            # and get silently dropped, causing async_read() to hold the stale
            # last-good-frame forever.
            self._last_seq = -1
            try:
                self._ws_app = websocket.WebSocketApp(
                    url,
                    on_open=self._on_open,
                    on_message=self._on_message,
                    on_close=self._on_close,
                    on_error=self._on_error,
                )
                self._ws_app.run_forever(ping_interval=10, ping_timeout=5)
            except Exception as e:  # pragma: no cover — defensive
                logger.warning("XELA reader unexpected exception: %s", e)
            if self._stop.is_set():
                break
            logger.warning("XELA WS disconnected; reconnecting in %.2fs", backoff)
            self._stop.wait(timeout=backoff)
            backoff = min(backoff * 2.0, _MAX_BACKOFF_S)

    def _on_open(self, ws):  # noqa: ANN001
        logger.info("XELA WS connected (%s:%d)", self.config.host, self.config.port)

    def _on_message(self, ws, message):  # noqa: ANN001
        # Parse + validate without holding the lock.
        try:
            payload = json.loads(message)
        except Exception as e:
            logger.warning("XELA: malformed JSON, dropping frame: %s", e)
            return
        if is_welcome(payload):
            return
        try:
            result = parse_frame(message, sensor_id=self.config.sensor_id,
                                 expected_model=self.config.model)
        except Exception as e:
            logger.warning("XELA: parse error, dropping frame: %s", e)
            return
        if result.seq <= self._last_seq:
            return  # out-of-order, drop
        self._last_seq = result.seq
        with self._lock:
            self._latest = result.raw
            self._latest_cal = result.calibrated
            self._latest_ts = result.timestamp
            if self.config.tare_on_connect and self._tare_offset is None:
                self._tare_offset = result.raw.copy()
        self._frame_event.set()

    def _on_close(self, ws, status, msg):  # noqa: ANN001
        # Demote to INFO when we asked for the close (graceful disconnect()).
        # Only WARNING when the server-side or transport closed unexpectedly.
        if self._stop.is_set():
            logger.info("XELA WS closed cleanly (status=%s)", status)
        else:
            logger.warning("XELA WS closed unexpectedly (status=%s, msg=%s)", status, msg)

    def _on_error(self, ws, error):  # noqa: ANN001
        logger.warning("XELA WS error: %s", error)

    # ----- standalone smoke entrypoint -----

    @staticmethod
    def _cli() -> None:
        """Run as ``python -m lerobot.tactile.xela.xela_tactile --host ...`` for smoke testing."""
        import argparse

        ap = argparse.ArgumentParser(description="XELA tactile sensor smoke test.")
        ap.add_argument("--host", default="auto",
                        help="XELA server IP. Default 'auto' resolves to the host's "
                             "primary LAN IP (matches xela_server's bind behavior).")
        ap.add_argument("--port", type=int, default=5000)
        ap.add_argument("--sensor-id", default="1")
        ap.add_argument("--model", default="XR1944")
        ap.add_argument("--seconds", type=float, default=5.0)
        args = ap.parse_args()

        logging.basicConfig(level=logging.INFO)
        cfg = XelaTactileConfig(host=args.host, port=args.port,
                                sensor_id=args.sensor_id, model=args.model)
        sensor = XelaTactileSensor(cfg)
        sensor.connect()
        try:
            t_end = time.time() + args.seconds
            while time.time() < t_end:
                frame = sensor.async_read()
                print(f"min={frame.min():8.1f} max={frame.max():8.1f} "
                      f"mean={frame.mean():8.2f} ts={sensor.latest_timestamp}")
                time.sleep(0.1)
        finally:
            sensor.disconnect()


if __name__ == "__main__":
    XelaTactileSensor._cli()
