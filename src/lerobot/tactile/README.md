# Tactile Sensors

Subsystem for streaming tactile readings into LeRobot observation dicts and
recording them as standalone vector features in `LeRobotDataset`.

## What lives here

```
src/lerobot/tactile/
├── configs.py          # TactileSensorConfig — abstract base, draccus ChoiceRegistry root
├── tactile_sensor.py   # TactileSensor — abstract Protocol every backend implements
├── utils.py            # make_tactile_sensors_from_configs — lazy backend factory
├── mock/               # MockTactileSensor — synthetic sinusoidal frames for CI / offline dev
└── xela/               # XelaTactileSensor — WebSocket client for xela_server v1.7.x
```

The two existing backends are registered with the same draccus
`ChoiceRegistry` so `--robot.tactile_sensors='{name: {type: xela|mock, ...}}'`
selects between them at the command line. Backend config classes are
**eager-imported** by [`__init__.py`](__init__.py) so their
`@register_subclass` decorators run before draccus parses CLI args; sensor
classes themselves stay lazy via `make_tactile_sensors_from_configs` (e.g.,
the XELA backend only pulls in `websocket-client` when actually instantiated).

## Data contract

Every `TactileSensor.async_read()` returns a flat 1-D `np.float32` array. The
shape is backend-specific (e.g., XELA `XR1944` → `(48,)`, i.e. 16 taxels × 3
axes). Robots wire the read into `obs_dict[f"observation.tactile.{name}"]`,
where `name` is the descriptive key from `--robot.tactile_sensors='{name: …}'`.

The recording pipeline preserves this 1-D shape end-to-end:

| Stage                   | Spec                                                               |
| ----------------------- | ------------------------------------------------------------------ |
| Robot observation key   | `observation.tactile.<name>`                                       |
| Raw value               | `np.ndarray` shape `(N,)`, dtype `float32`                         |
| Dataset feature         | `{"dtype": "float32", "shape": (N,), "names": ["0", …, str(N-1)]}` |
| `LeRobotDataset` column | Same; each frame stored as one Parquet `list<float32>` cell        |

This is distinct from images (`(H, W, 3)`, dtype `video`/`image`) and bundled
state vectors (`observation.state`, packed from individual scalar joint keys).
The split is enforced in
[`utils/feature_utils.py:hw_to_dataset_features`](../../utils/feature_utils.py)
— 3-tuple shapes go to the camera/video bucket, 1-tuple shapes are emitted
under their original key as standalone vector features.

## Recovering the 2D spatial layout (reshape contract)

The flat `(N,)` shape is a **storage** choice, not a structural one. The data
preserves the sensor's physical 4×4 (or 4×6) taxel grid and the 3 force-axis
components at each taxel — both can be recovered by a single
`numpy.reshape` with no data movement and no information loss.

**The contract for XELA sensors** (XR1944, XR1946, XR2244, XR1922):

```python
import numpy as np

flat = obs["observation.tactile.right_finger_r"]   # shape (N,), dtype float32

# XR1944: 4×4 taxels × 3 axes (X, Y, Z magnetic-field components per taxel)
grid = flat.reshape(4, 4, 3)
# After reshape:
#   grid[row, col, 0] = X-axis reading at taxel (row, col)
#   grid[row, col, 1] = Y-axis reading at taxel (row, col)
#   grid[row, col, 2] = Z-axis reading at taxel (row, col)
#
# Row order: top → bottom; column order: left → right when the sensor is held
# with the cable exiting on the right (XELA Software Manual v1.7.6 §"Sensor
# Layout", p. 36: "data is read from top left towards right, line-by-line").
#
# Concretely, taxel index i (0..15) maps to (row=i//4, col=i%4), so:
#   flat[i*3 + a] == grid[i // 4, i % 4, a]   for a ∈ {0,1,2}
```

For **XR1946** the same contract holds with `flat.reshape(4, 6, 3)`. The rule:
`flat.reshape(taxels_rows, taxels_cols, 3)` where `(taxels_rows,
taxels_cols)` is the physical layout from the manual.

### Why we store flat, not as `(rows, cols, 3)`

LeRobot's recording pipeline (`feature_utils.hw_to_dataset_features`)
dispatches feature shapes to one of two buckets: 3-tuple → camera/video
encoding (h264/hevc, requires ≥16 px sides), 1-tuple → numeric vector. There
is currently no multi-dim numeric-tensor bucket, so a `(4, 4, 3)` shape would
fail-route into the video path and break recording. Adding such a bucket is a
framework-level change tracked separately from this subsystem; until that
lands, the `(N,)` + reshape contract is the working storage shape.

### When you need the 2D structure

- **MLP / state-encoder policies** (ACT, π0, SmolVLA tactile branches that go
  through a small dense net): consume `flat` directly. The reshape gives no
  benefit — the first dense layer collapses everything anyway.
- **CNN / ViT tactile encoders** (small Conv2d over the 4×4 grid, or "tactile
  as patch tokens"): apply `flat.reshape(rows, cols, 3)` inside the policy's
  forward pass. The structural prior (row/col adjacency, per-taxel 3-channel
  features) is preserved and the layer benefits from translation equivariance
  across the pad.
- **Visualization / debugging**: the inspection utility
  [`examples/tactile/inspect_tactile_dataset.py`](../../../examples/tactile/inspect_tactile_dataset.py)
  already applies this reshape internally for the `heatmap` and `frame`
  modes — read its source for a concrete reference implementation.

## Recorded-key naming convention

`observation.tactile.<descriptive_name>`. We use `<arm>_finger_<jaw>` for the
bi_yam follower (e.g., `right_finger_r` = right-jaw fingertip pad of the
right-arm gripper), but the key is opaque to the recorder — any string works.

Add additional pads by extending `--robot.tactile_sensors='{...}'`; each entry
becomes its own `observation.tactile.<key>` column automatically.

## Adding a new backend

1. Add a config class in a new package (e.g., `src/lerobot/tactile/<vendor>/configuration_<vendor>.py`)
   that subclasses `TactileSensorConfig` and is decorated with
   `@TactileSensorConfig.register_subclass("<vendor>")`.
2. Add a sensor class implementing the `TactileSensor` Protocol from
   [`tactile_sensor.py`](tactile_sensor.py): `connect()`, `disconnect()`,
   `is_connected`, `shape`, `async_read()`.
3. Eager-import the **config** from this directory's
   [`__init__.py`](__init__.py) so the decorator runs at package load —
   sensor classes (which may pull in optional deps) stay lazy via
   `make_tactile_sensors_from_configs`.
4. Add a dispatch branch in `utils.make_tactile_sensors_from_configs` that
   imports your sensor class on demand.

The XELA backend is the canonical reference implementation — see
[`xela/README.md`](xela/README.md).

## See also

- [`xela/README.md`](xela/README.md) — XELA backend wire protocol, vendor
  software first-time setup, per-boot bring-up, optional vendor tools
  (`xela_viz`, `xela_log`), and a comprehensive failure-modes reference
  (LeRobot client side + vendor binary side).
- [`examples/tactile/README.md`](../../../examples/tactile/README.md)
  — `inspect_tactile_dataset.py` for verifying schema and visualising
  recorded episodes.
- [`../robots/bi_yam_follower/README.md`](../robots/bi_yam_follower/README.md#tactile-sensor-xela-optional)
  — recording-side usage on the bimanual Yam follower (the only robot
  currently wired up to expose `--robot.tactile_sensors`).
