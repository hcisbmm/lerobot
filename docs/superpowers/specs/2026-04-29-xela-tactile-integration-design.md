# XELA Tactile Integration for Bimanual YAM — Design

Status: draft (2026-04-29)
Owner: hcisbmm
Target: bimanual YAM teleoperation + LeRobot dataset recording with one XR1944 tactile pad on the right-side fingertip of the right follower's parallel gripper.

## Problem

LeRobot's `BiYamFollower` records joint state, effort, and camera frames. The lab has a XELA Robotics XR1944 (4×4 taxel × 3 axes = 48 channels) tactile sensor mounted on the **right-side fingertip of the right arm's parallel gripper** (not the palm), served by XELA Server v1.7.6 binaries already installed under `/etc/xela/`. We need to plumb tactile readings into the LeRobot observation dict so they ride along in episodes recorded by `lerobot-record`, with a clean abstraction that future robots / additional sensors can reuse.

**Observation-key naming convention (forward-compatible):** `observation.tactile.<arm>_finger_<side>` where `<arm> ∈ {left, right}` (which YAM arm) and `<side> ∈ {l, r}` (which jaw of that parallel gripper). This sensor → `right_finger_r`. Adding more sensors later (left arm, opposite jaw) is just another entry in `tactile_sensors` — no schema rename.

## Constraints (verified)

- **Hardware:** one XR1944 board on `slcan0` (`/etc/xela/xServ.ini` shows `num_brd=1`, `model=XR1944`, `channel=0`, `bustype=socketcan`, `channel=slcan0`).
- **Wire protocol:** XELA Server pushes JSON frames over a WebSocket (`ws://<ip>:<port>`, default port 5000), confirmed by the v1.7.6 manual and an observed disconnect line in `/etc/xela/LOG/xela_server.log`. **Empirically the AppImage v1.7.6 build 158509 silently ignores `--ip` and always binds to the host's primary NIC IP** (e.g., 192.168.x.x), so we cannot pin to 127.0.0.1. The `XelaTactileConfig.host` default is `"auto"`, which `XelaTactileSensor` resolves at connect time to the same outbound-route IP `xela_server` uses (UDP socket trick to discover the route to a public IP).
- **Sample rate:** ~102 Hz observed in `xela_server.log` (`[PCHECK]…102.5Hz`). Teleop loop is 30 Hz → consumer is sample-and-hold.
- **Frame schema** (manual p. 37):
  ```
  {"message": <seq>, "time": <unix_ts>, "sensors": <K>,
   "1": {"data": "FA00,FB00,FC00,...", "sensor": "1", "taxels": 16,
         "model": "XR1944", "calibrated": null | [floats]}}
  ```
  `data` is comma-separated, leading-zero-stripped hex, length `3 * taxels` (X,Y,Z per taxel).
- **First message:** `{"message":"Welcome", ...}` — must be skipped.
- **Bring-up:** `slcand -o -s8 -t hw -S 3000000 /dev/ttyUSB0 slcan0` → `ifconfig slcan0 up` → (one-shot) `xela_conf -d socketcan -c slcan0` → `xela_server -f /etc/xela/xServ.ini -p 5000 --noros` (we omit `--ip` since v1.7.6 ignores it; the server binds to the LAN IP and our client auto-resolves to the same one).
- **Python client dependency:** `websocket-client` (manual p. 9 prereq).

## Decisions

1. **Q1 sensor placement:** one XR1944, mounted on the right-side fingertip of the right follower's parallel gripper. Single-arm, single-finger tactile, asymmetric data.
2. **Q2 transport:** native WebSocket client; no Python SDK dependency, no ROS.
3. **Q3 observation schema:** flat float vector under one packed key (`observation.tactile.right_finger_r`, shape `(48,)`, dtype `float32`). No image-style framing.
4. **Q4 placement:** new top-level subsystem `src/lerobot/tactile/`, parallel to `src/lerobot/cameras/`. Robots opt-in via a `tactile_sensors: dict[str, TactileSensorConfig]` field.
5. **Lifecycle:** `xela_server` runs externally (mirrors how `run_bimanual_yam_server.py` is launched). LeRobot connects as a client.
6. **Default record content:** raw 48-channel vector (uint16 magnetic-field readings cast losslessly to `float32`, since uint16 ⊂ float32). With `use_calibrated=True`, an additional `(48,)` `float32` `*.cal` column is emitted from each frame's `calibrated` field (XCAL forces in Newtons). If XCAL files are not installed server-side, the `.cal` column is zero-filled and a one-shot `ERROR` is logged.
7. **Tare:** off by default; opt-in flag `tare_on_connect` for users who want it.
8. **Failure mode:** WS disconnect → log warning, return last-good-frame, auto-reconnect with capped exponential backoff.
9. **No new `lerobot-record-with-tactile` script.** Stock `lerobot-record` works because tactile flows through `BiYamFollower.get_observation()`.

## Architecture

```
src/lerobot/tactile/
├── __init__.py
├── tactile_sensor.py      # TactileSensor abstract base
├── configs.py             # TactileSensorConfig base + draccus.ChoiceRegistry
├── utils.py               # make_tactile_sensors_from_configs(...)
├── mock/
│   └── mock_tactile.py    # MockTactileSensor (sinusoidal fake, for CI)
└── xela/
    ├── __init__.py
    ├── configuration_xela.py  # XelaTactileConfig (registers as "xela")
    ├── xela_tactile.py        # XelaTactileSensor (WebSocket client)
    └── README.md              # bring-up + xServ.ini reference

src/lerobot/robots/bi_yam_follower/
├── run_xela_server.py     # bring-up wrapper (slcan up + xela_server invocation)
├── config_bi_yam_follower.py  # +tactile_sensors field
├── bi_yam_follower.py     # +parallel-read of tactile_sensors in get_observation()
└── README.md              # +tactile section

pyproject.toml             # +websocket-client to [yam] extra
tests/tactile/             # unit + integration tests
```

## Public API

```python
class TactileSensor(ABC):
    config_class: type[TactileSensorConfig]
    name: str
    @abstractmethod def connect(self) -> None: ...
    @abstractmethod def disconnect(self) -> None: ...
    @property @abstractmethod def is_connected(self) -> bool: ...
    @property @abstractmethod def shape(self) -> tuple[int, ...]: ...   # (48,) for XR1944
    @property @abstractmethod def dtype(self) -> np.dtype: ...
    @abstractmethod def async_read(self) -> np.ndarray: ...
    @property def latest_timestamp(self) -> float | None: ...           # XELA `time` field
```

```python
@dataclass
@TactileSensorConfig.register_subclass("xela")
class XelaTactileConfig(TactileSensorConfig):
    type: str = "xela"
    host: str = "auto"  # resolved at connect time to the host's primary LAN IP
    port: int = 5000
    sensor_id: str = "1"
    model: str = "XR1944"
    use_calibrated: bool = False
    tare_on_connect: bool = False
    reconnect_backoff_s: float = 0.5
    receive_timeout_s: float = 1.0
```

`SHAPE_BY_MODEL = {"XR1944": (16, 3), "XR1946": (24, 3), "XR2244": (16, 3)}` — flattened length is the public shape.

## Data flow per teleop tick

```
xela_server (slcan→board) ──ws JSON ~100 Hz──▶ XelaTactileSensor._reader_thread
                                                       │
                                                       ▼ (Lock + Event)
                                                latest_frame: np.ndarray((48,), float32)
                                                       ▲
BiYamFollower.get_observation()
  └─ executor.submit(sensor.async_read) ──────────────┘
            │
            ▼
   obs["observation.tactile.right_finger_r"]   # shape (48,) float32
```

The reader thread holds a `WebSocketApp` open; on each frame it parses, validates `seq` is monotonic, writes to a single-slot buffer guarded by a `threading.Lock`, and signals an `Event` for `async_read` to wake (with a `receive_timeout_s` ceiling).

## Observation schema

| Key                                      | Shape   | Dtype     | When                                                                                                                                                                                                             |
| ---------------------------------------- | ------- | --------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `observation.tactile.right_finger_r`     | `(48,)` | `float32` | Always when sensor is configured. Decoded from comma-separated 4-char hex (uint16 in `[0, 65535]`) and cast losslessly to `float32`. Matches the dtype of all other proprio columns; zero cast cost in training. |
| `observation.tactile.right_finger_r.cal` | `(48,)` | `float32` | When `use_calibrated=True`. XCAL-calibrated forces (Newtons). If XCAL files are missing server-side, zero-filled with a one-shot `ERROR` in the log.                                                             |

`BiYamFollower.observation_features` exposes `("observation.tactile.right_finger_r", (48,))` so `LeRobotDataset` writes it as a numeric tensor column — no video codec involvement.

## WebSocket parser (XelaTactileSensor)

Per the manual (pp. 37–38):

1. Connect to `ws://host:port`.
2. First message: ignore if `data["message"] == "Welcome"`.
3. Each subsequent message:
   - `seq = data["message"]` (int) — drop frames where `seq <= last_seq` (handles re-orderings).
   - `frame = data[sensor_id]`.
   - Assert `frame["model"] == config.model` once at first frame; raise on mismatch.
   - `raw = np.array([int(s, 16) for s in frame["data"].split(",")], dtype=np.float32)` — length must equal `3 * taxels`. (uint16 values fit losslessly in float32.)
   - If `use_calibrated` and `frame["calibrated"]` is a list, also store `np.asarray(frame["calibrated"], dtype=np.float32)`.
   - Atomic swap into `_latest`.

## Bring-up runbook (excerpt added to bi_yam_follower/README.md)

```bash
# (one-time on each boot) bring up the SLCAN bus
sudo slcand -o -s8 -t hw -S 3000000 /dev/ttyUSB0 slcan0
sudo ifconfig slcan0 up

# Terminal X — start XELA server (leave running)
/etc/xela/xela_server -f /etc/xela/xServ.ini -p 5000 --noros &
# Note: --ip is omitted intentionally — the AppImage ignores it and binds to the
# host's primary NIC IP. Terminate later with `kill $!` or `pkill -f xela_server`.

# Terminal Y — verify the stream is alive (optional)
python -m lerobot.tactile.xela.xela_tactile --port 5000 --sensor-id 1
# `--host` defaults to "auto" → resolves to the same LAN IP xela_server bound to.

# Terminal Z — teleop or record with tactile
lerobot-record \
  --robot.type=bi_yam_follower \
  --robot.tactile_sensors='{
     right_finger_r: {"type":"xela","port":5000,
                  "sensor_id":"1","model":"XR1944"}
  }' \
  --teleop.type=bi_yam_leader \
  --dataset.repo_id="${HF_USER}/bimanual-yam-tactile-demo" \
  --dataset.single_task="Tactile pick and place" \
  --fps=30
```

`run_xela_server.py` wraps the slcan + server invocation behind a single command (`python src/lerobot/robots/bi_yam_follower/run_xela_server.py`).

## Failure modes & guarantees

| Condition                                               | Behaviour                                                                                                                                         |
| ------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------- |
| `xela_server` not reachable at `connect()`              | Raise `ConnectionError` with bring-up runbook hint.                                                                                               |
| WS disconnect mid-episode                               | Reader auto-reconnects (backoff `reconnect_backoff_s` doubling, capped at 2 s); `async_read` returns last-good-frame; one warning per disconnect. |
| Sequence regression                                     | Frame dropped silently.                                                                                                                           |
| Stale frame (`now − latest_timestamp > 1 s`)            | One-shot WARNING per staleness episode; data still returned.                                                                                      |
| Model mismatch (xServ.ini vs config)                    | `connect()` raises `RuntimeError`.                                                                                                                |
| `use_calibrated=True` but `frame["calibrated"]` is null | One-shot WARNING; `*.cal` key omitted from observation that tick.                                                                                 |

## Testing

- **Unit** (`tests/tactile/test_xela_parser.py`): hex-CSV decoder, sequence dedupe, calibrated-vs-raw paths, model-mismatch error.
- **Unit** (`tests/tactile/test_mock.py`): `MockTactileSensor` produces correct shape/dtype.
- **Integration** (`tests/tactile/test_record_with_mock_tactile.py`): record 2 short episodes with `BiYamFollower` (mocked arms) + `MockTactileSensor`; assert dataset has `observation.tactile.right_finger_r` column with correct shape and frame count.
- **Hardware smoke** (manual): `xela_tactile.py` standalone CLI prints decoded frames; verify ~30 Hz consumer matches ~100 Hz producer (no buffering creep).

## Out of scope

- Multi-board xServ.ini (single sensor today; `sensor_id` field stays for forward compatibility).
- ROS bridge (`--noros` always).
- Live tactile visualisation in Rerun (XELA's own `xela_viz` is sufficient; can revisit by deriving a 64×64 colorized heatmap later, computed from the stored vector — no schema change).
- Auto-launching `xela_server` from inside `BiYamFollower.connect()`.
- Tactile on left arm or additional pads (forward-compatible — just add another entry to `tactile_sensors`).

## File-level diff summary

| Action | File                                                           | LOC est. |
| ------ | -------------------------------------------------------------- | -------- |
| New    | `src/lerobot/tactile/__init__.py`                              | 5        |
| New    | `src/lerobot/tactile/configs.py`                               | 30       |
| New    | `src/lerobot/tactile/tactile_sensor.py`                        | 60       |
| New    | `src/lerobot/tactile/utils.py`                                 | 25       |
| New    | `src/lerobot/tactile/xela/configuration_xela.py`               | 35       |
| New    | `src/lerobot/tactile/xela/xela_tactile.py`                     | 220      |
| New    | `src/lerobot/tactile/mock/mock_tactile.py`                     | 60       |
| New    | `src/lerobot/robots/bi_yam_follower/run_xela_server.py`        | 80       |
| Edit   | `src/lerobot/robots/bi_yam_follower/config_bi_yam_follower.py` | +15      |
| Edit   | `src/lerobot/robots/bi_yam_follower/bi_yam_follower.py`        | +50      |
| Edit   | `src/lerobot/robots/bi_yam_follower/README.md`                 | +60      |
| Edit   | `pyproject.toml` (add `websocket-client` to `[yam]` extra)     | +1       |
| New    | `tests/tactile/`                                               | ~150     |
