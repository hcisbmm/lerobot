# XELA Tactile Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Plumb a single XELA XR1944 tactile sensor (mounted on the right-side fingertip of the right YAM follower's parallel gripper) into LeRobot's `BiYamFollower` observation dict, with a clean reusable `src/lerobot/tactile/` subsystem that mirrors `src/lerobot/cameras/`.

**Architecture:** New top-level subsystem `src/lerobot/tactile/` with a `TactileSensor` ABC, a `TactileSensorConfig` registry, a `MockTactileSensor` for CI, and an `XelaTactileSensor` WebSocket client backend. `BiYamFollowerConfig` gains a `tactile_sensors` field; `BiYamFollower.get_observation()` reads tactile sensors in parallel alongside cameras. `xela_server` runs externally; LeRobot is a pure client. Spec: [docs/superpowers/specs/2026-04-29-xela-tactile-integration-design.md](../specs/2026-04-29-xela-tactile-integration-design.md).

**Tech Stack:** Python 3.12+, `numpy`, `dataclasses`, `draccus.ChoiceRegistry`, `websocket-client` (new dep), `threading`, `pytest`, `unittest.mock`.

---

## File Structure

| Path                                                           | Responsibility                                                                               | New/Edit |
| -------------------------------------------------------------- | -------------------------------------------------------------------------------------------- | -------- |
| `src/lerobot/tactile/__init__.py`                              | Public surface for the subsystem (re-exports).                                               | New      |
| `src/lerobot/tactile/configs.py`                               | `TactileSensorConfig` base dataclass + `draccus.ChoiceRegistry`.                             | New      |
| `src/lerobot/tactile/tactile_sensor.py`                        | `TactileSensor` ABC.                                                                         | New      |
| `src/lerobot/tactile/utils.py`                                 | `make_tactile_sensors_from_configs(...)` factory.                                            | New      |
| `src/lerobot/tactile/mock/__init__.py`                         | Re-export `MockTactileSensor`, `MockTactileConfig`.                                          | New      |
| `src/lerobot/tactile/mock/mock_tactile.py`                     | `MockTactileConfig` + `MockTactileSensor` (deterministic sinusoid generator, no I/O).        | New      |
| `src/lerobot/tactile/xela/__init__.py`                         | Re-export `XelaTactileSensor`, `XelaTactileConfig`.                                          | New      |
| `src/lerobot/tactile/xela/configuration_xela.py`               | `XelaTactileConfig` dataclass + register_subclass.                                           | New      |
| `src/lerobot/tactile/xela/parser.py`                           | Pure functions for parsing XELA WebSocket frames (no I/O, fully unit-testable).              | New      |
| `src/lerobot/tactile/xela/xela_tactile.py`                     | `XelaTactileSensor` (WebSocket reader thread, last-good-frame, reconnect).                   | New      |
| `src/lerobot/tactile/xela/README.md`                           | XELA bring-up notes.                                                                         | New      |
| `src/lerobot/utils/import_utils.py`                            | Add `_websocket_client_available` flag.                                                      | Edit     |
| `src/lerobot/robots/bi_yam_follower/config_bi_yam_follower.py` | Add `tactile_sensors: dict[str, TactileSensorConfig] = {}` field.                            | Edit     |
| `src/lerobot/robots/bi_yam_follower/bi_yam_follower.py`        | Wire `tactile_sensors` into `connect`/`disconnect`/`observation_features`/`get_observation`. | Edit     |
| `src/lerobot/robots/bi_yam_follower/run_xela_server.py`        | Operator wrapper that launches `xela_server` with sensible defaults.                         | New      |
| `src/lerobot/robots/bi_yam_follower/README.md`                 | Add tactile bring-up section.                                                                | Edit     |
| `pyproject.toml`                                               | Add `websocket-client>=1.7,<2.0` to `[project.optional-dependencies] yam`.                   | Edit     |
| `tests/tactile/__init__.py`                                    | Empty package marker.                                                                        | New      |
| `tests/tactile/test_configs.py`                                | Test config registry round-trip.                                                             | New      |
| `tests/tactile/test_factory.py`                                | Test `make_tactile_sensors_from_configs`.                                                    | New      |
| `tests/tactile/test_mock.py`                                   | Test `MockTactileSensor`.                                                                    | New      |
| `tests/tactile/test_xela_parser.py`                            | Golden-frame tests for parser (manual page 37 examples).                                     | New      |
| `tests/tactile/test_xela_sensor.py`                            | `XelaTactileSensor` tests with mocked WebSocketApp.                                          | New      |
| `tests/robots/test_bi_yam_tactile.py`                          | `BiYamFollower` × `MockTactileSensor` integration.                                           | New      |

---

## Conventions

- Every new Python file starts with the standard Apache-2.0 header used in the rest of the repo (copy verbatim from any existing file like [src/lerobot/cameras/camera.py:1-15](../../src/lerobot/cameras/camera.py)).
- Tests run with `uv run pytest <path> -v`.
- Commit message prefix: `feat(tactile):` for code, `test(tactile):` for tests, `docs(tactile):` for docs, `chore(tactile):` for build/config.
- All numpy arrays in this subsystem are `np.float32` unless explicitly stated.

---

### Task 1: Add `websocket-client` dependency and import-availability flag

**Files:**

- Modify: `pyproject.toml:172-176`
- Modify: `src/lerobot/utils/import_utils.py:110-124`
- Test: `tests/tactile/__init__.py`, `tests/tactile/test_imports.py`

- [ ] **Step 1: Create the test directory and write the failing import-flag test**

Create `tests/tactile/__init__.py` (empty file).

Create `tests/tactile/test_imports.py`:

```python
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

from lerobot.utils import import_utils


def test_websocket_client_flag_exists():
    """The tactile subsystem needs a guarded availability flag for websocket-client."""
    assert hasattr(import_utils, "_websocket_client_available")
    assert isinstance(import_utils._websocket_client_available, bool)
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `uv run pytest tests/tactile/test_imports.py -v`
Expected: FAIL with `AttributeError: module 'lerobot.utils.import_utils' has no attribute '_websocket_client_available'`

- [ ] **Step 3: Add the import-availability flag**

Edit `src/lerobot/utils/import_utils.py`. Find the "Hardware SDKs" block (line ~110-124) and add a new line right after the `_portal_available` line:

```python
_websocket_client_available = is_package_available("websocket-client", import_name="websocket")
```

(The package is named `websocket-client` on PyPI but is imported as `websocket`.)

- [ ] **Step 4: Run the test — should now pass**

Run: `uv run pytest tests/tactile/test_imports.py -v`
Expected: PASS

- [ ] **Step 5: Add `websocket-client` to the `yam` optional-dependency extra**

Edit `pyproject.toml`. Find the `yam = [...]` block at line 172 and update it:

```toml
yam = [
    "portal>=3.7.0,<4.0.0 ; sys_platform == 'linux'",
    "i2rt ; sys_platform == 'linux'",
    "mujoco>=3.2.0,<4.0.0",
    "websocket-client>=1.7,<2.0 ; sys_platform == 'linux'",
]
```

- [ ] **Step 6: Sync the lockfile and verify the package is installable**

Run: `uv sync --locked --extra yam`
Expected: clean exit, no resolution errors. (If this is a workspace where `--locked` rejects the new dep because the lockfile predates it, run `uv sync --extra yam` instead and commit the regenerated `uv.lock`.)

Run: `uv run python -c "import websocket; print(websocket.__version__)"`
Expected: prints a version like `1.8.0`.

- [ ] **Step 7: Commit**

```bash
git add tests/tactile/__init__.py tests/tactile/test_imports.py \
        src/lerobot/utils/import_utils.py pyproject.toml uv.lock
git commit -m "chore(tactile): add websocket-client to yam extra and import flag"
```

---

### Task 2: `TactileSensorConfig` base class

**Files:**

- Create: `src/lerobot/tactile/__init__.py`
- Create: `src/lerobot/tactile/configs.py`
- Test: `tests/tactile/test_configs.py`

- [ ] **Step 1: Write the failing test**

Create `tests/tactile/test_configs.py`:

```python
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

from dataclasses import dataclass

import draccus

from lerobot.tactile.configs import TactileSensorConfig


def test_base_config_is_choice_registry():
    """TactileSensorConfig must be a draccus.ChoiceRegistry so subclasses can register."""
    assert issubclass(TactileSensorConfig, draccus.ChoiceRegistry)


def test_subclass_registration_and_type_property():
    """A registered subclass must report its registered name via the `type` property."""

    @TactileSensorConfig.register_subclass("dummy_tactile_for_test")
    @dataclass
    class _DummyConfig(TactileSensorConfig):
        x: int = 1

    cfg = _DummyConfig(x=2)
    assert cfg.type == "dummy_tactile_for_test"
    assert cfg.x == 2
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `uv run pytest tests/tactile/test_configs.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'lerobot.tactile'`.

- [ ] **Step 3: Create `src/lerobot/tactile/__init__.py`**

```python
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

from .configs import TactileSensorConfig
from .tactile_sensor import TactileSensor

__all__ = ["TactileSensor", "TactileSensorConfig"]
```

(`tactile_sensor` doesn't exist yet — that's fine; it gets imported lazily once Task 3 lands. Keep this `__init__` as the long-term shape so we don't churn it.)

- [ ] **Step 4: Create `src/lerobot/tactile/configs.py`**

```python
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
from dataclasses import dataclass

import draccus  # type: ignore  # TODO: add type stubs for draccus


@dataclass(kw_only=True)
class TactileSensorConfig(draccus.ChoiceRegistry, abc.ABC):  # type: ignore  # TODO: add type stubs for draccus
    """Base config for tactile sensors. Backends register via `@TactileSensorConfig.register_subclass(name)`."""

    @property
    def type(self) -> str:
        return str(self.get_choice_name(self.__class__))
```

- [ ] **Step 5: Make Task 3 import work — add a stub `tactile_sensor.py`**

Because `__init__.py` imports `TactileSensor`, we need at least an empty placeholder; we'll fill it in Task 3. Create `src/lerobot/tactile/tactile_sensor.py`:

```python
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

# Filled in by Task 3.
class TactileSensor:
    pass
```

- [ ] **Step 6: Run the test — should pass**

Run: `uv run pytest tests/tactile/test_configs.py -v`
Expected: PASS (both `test_base_config_is_choice_registry` and `test_subclass_registration_and_type_property`).

- [ ] **Step 7: Commit**

```bash
git add src/lerobot/tactile/__init__.py src/lerobot/tactile/configs.py \
        src/lerobot/tactile/tactile_sensor.py tests/tactile/test_configs.py
git commit -m "feat(tactile): add TactileSensorConfig base with draccus ChoiceRegistry"
```

---

### Task 3: `TactileSensor` ABC

**Files:**

- Modify: `src/lerobot/tactile/tactile_sensor.py`
- Test: `tests/tactile/test_tactile_sensor.py`

- [ ] **Step 1: Write the failing test**

Create `tests/tactile/test_tactile_sensor.py`:

```python
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

    s = _Stub()
    assert s.latest_timestamp is None
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `uv run pytest tests/tactile/test_tactile_sensor.py -v`
Expected: FAIL — `TactileSensor` has no abstract methods yet, the placeholder `class TactileSensor: pass` is not abstract.

- [ ] **Step 3: Replace the stub with the real ABC**

Replace the contents of `src/lerobot/tactile/tactile_sensor.py`:

```python
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

import numpy as np
from numpy.typing import NDArray  # type: ignore  # TODO: add type stubs for numpy.typing

from .configs import TactileSensorConfig


class TactileSensor(abc.ABC):
    """Base class for tactile sensor backends.

    Mirrors the role of `lerobot.cameras.Camera` for tactile pads. Backends produce a
    flat 1-D float32 vector per read (e.g., 48 channels for an XR1944: 16 taxels × 3 axes).
    """

    def __init__(self, config: TactileSensorConfig):
        self.config = config

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.disconnect()

    def __del__(self) -> None:
        try:
            if self.is_connected:
                self.disconnect()
        except Exception:  # nosec B110
            pass

    @abc.abstractmethod
    def connect(self) -> None:
        """Open the underlying transport (network / driver) and start reading."""

    @abc.abstractmethod
    def disconnect(self) -> None:
        """Tear down the transport and stop any background reader."""

    @property
    @abc.abstractmethod
    def is_connected(self) -> bool:
        """True if the sensor is currently connected and producing frames."""

    @property
    @abc.abstractmethod
    def shape(self) -> tuple[int, ...]:
        """Shape of a single observation, e.g. `(48,)` for XR1944."""

    @property
    @abc.abstractmethod
    def dtype(self) -> np.dtype:
        """Numpy dtype of an observation; expected to be `np.float32` for this subsystem."""

    @abc.abstractmethod
    def async_read(self) -> NDArray[np.float32]:
        """Return the latest observation, non-blocking.

        Backends following the spec's failure-mode rules return the last-good-frame
        if the underlying transport is mid-reconnect.
        """

    @property
    def latest_timestamp(self) -> float | None:
        """Wall-clock timestamp (seconds since epoch) of the latest frame, or None."""
        return None
```

- [ ] **Step 4: Run the test — should pass**

Run: `uv run pytest tests/tactile/test_tactile_sensor.py -v`
Expected: PASS for all three test functions.

- [ ] **Step 5: Run the whole tactile test directory to confirm nothing regressed**

Run: `uv run pytest tests/tactile/ -v`
Expected: all tests so far PASS (imports + configs + tactile_sensor).

- [ ] **Step 6: Commit**

```bash
git add src/lerobot/tactile/tactile_sensor.py tests/tactile/test_tactile_sensor.py
git commit -m "feat(tactile): add TactileSensor ABC base class"
```

---

### Task 4: `make_tactile_sensors_from_configs` factory

**Files:**

- Create: `src/lerobot/tactile/utils.py`
- Test: `tests/tactile/test_factory.py`

- [ ] **Step 1: Write the failing test (factory should accept an empty dict)**

Create `tests/tactile/test_factory.py`:

```python
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
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `uv run pytest tests/tactile/test_factory.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'lerobot.tactile.utils'`.

- [ ] **Step 3: Create `src/lerobot/tactile/utils.py`**

```python
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

from .configs import TactileSensorConfig
from .tactile_sensor import TactileSensor


def make_tactile_sensors_from_configs(
    configs: dict[str, TactileSensorConfig],
) -> dict[str, TactileSensor]:
    """Instantiate tactile sensor backends from a name→config mapping.

    Lazy-imports backends so optional deps (e.g., `websocket-client` for XELA) only fail
    at the point of use, not at module load.
    """
    sensors: dict[str, TactileSensor] = {}
    for key, cfg in configs.items():
        if cfg.type == "mock":
            from .mock.mock_tactile import MockTactileSensor

            sensors[key] = MockTactileSensor(cfg)
        elif cfg.type == "xela":
            from .xela.xela_tactile import XelaTactileSensor

            sensors[key] = XelaTactileSensor(cfg)
        else:
            raise ValueError(
                f"Unsupported tactile sensor type {cfg.type!r} for key {key!r}. "
                f"Supported types: 'mock', 'xela'."
            )
    return sensors
```

- [ ] **Step 4: Run the test — partial pass expected**

Run: `uv run pytest tests/tactile/test_factory.py -v`
Expected: PASS — neither test path imports `mock` or `xela` modules; the empty-dict test and the unsupported-type test both succeed without those modules existing.

- [ ] **Step 5: Run the whole tactile test directory**

Run: `uv run pytest tests/tactile/ -v`
Expected: all PASS.

- [ ] **Step 6: Commit**

```bash
git add src/lerobot/tactile/utils.py tests/tactile/test_factory.py
git commit -m "feat(tactile): add make_tactile_sensors_from_configs factory"
```

---

### Task 5: `MockTactileSensor` (for CI and BiYamFollower integration tests)

**Files:**

- Create: `src/lerobot/tactile/mock/__init__.py`
- Create: `src/lerobot/tactile/mock/mock_tactile.py`
- Test: `tests/tactile/test_mock.py`

- [ ] **Step 1: Write failing tests for `MockTactileSensor`**

Create `tests/tactile/test_mock.py`:

```python
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
    a.connect(); b.connect()
    np.testing.assert_array_equal(a.async_read(), b.async_read())
    a.disconnect(); b.disconnect()


def test_factory_dispatches_to_mock_backend():
    sensors = make_tactile_sensors_from_configs({"left": MockTactileConfig()})
    assert isinstance(sensors["left"], MockTactileSensor)


def test_mock_sensor_latest_timestamp_after_read():
    sensor = MockTactileSensor(MockTactileConfig())
    sensor.connect()
    assert sensor.latest_timestamp is None
    sensor.async_read()
    assert isinstance(sensor.latest_timestamp, float)
    sensor.disconnect()
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `uv run pytest tests/tactile/test_mock.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'lerobot.tactile.mock'`.

- [ ] **Step 3: Create the mock package init**

Create `src/lerobot/tactile/mock/__init__.py`:

```python
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

from .mock_tactile import MockTactileConfig, MockTactileSensor

__all__ = ["MockTactileConfig", "MockTactileSensor"]
```

- [ ] **Step 4: Implement `MockTactileSensor` and `MockTactileConfig`**

Create `src/lerobot/tactile/mock/mock_tactile.py`:

```python
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

import time
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray  # type: ignore  # TODO: add type stubs for numpy.typing

from ..configs import TactileSensorConfig
from ..tactile_sensor import TactileSensor


@TactileSensorConfig.register_subclass("mock")
@dataclass
class MockTactileConfig(TactileSensorConfig):
    """Synthetic tactile sensor for CI and offline development.

    Generates deterministic-per-seed sinusoidal frames of shape ``(num_taxels * axes,)``.
    """

    num_taxels: int = 16
    axes: int = 3
    seed: int = 0


class MockTactileSensor(TactileSensor):
    """Deterministic synthetic tactile sensor — no I/O, safe for CI."""

    def __init__(self, config: MockTactileConfig):
        super().__init__(config)
        self.config: MockTactileConfig = config
        self._connected = False
        self._step = 0
        self._latest_timestamp: float | None = None
        self._n = config.num_taxels * config.axes
        self._rng = np.random.default_rng(config.seed)
        # Pre-bake per-channel phase offsets so each channel oscillates differently.
        self._phase = self._rng.uniform(0.0, 2 * np.pi, size=self._n).astype(np.float32)

    def connect(self) -> None:
        self._connected = True
        self._step = 0

    def disconnect(self) -> None:
        self._connected = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def shape(self) -> tuple[int, ...]:
        return (self._n,)

    @property
    def dtype(self) -> np.dtype:
        return np.dtype(np.float32)

    @property
    def latest_timestamp(self) -> float | None:
        return self._latest_timestamp

    def async_read(self) -> NDArray[np.float32]:
        if not self._connected:
            raise RuntimeError("MockTactileSensor is not connected; call connect() first.")
        t = self._step / 100.0  # nominal 100 Hz simulated time base
        frame = np.sin(2 * np.pi * t + self._phase, dtype=np.float32)
        self._step += 1
        self._latest_timestamp = time.time()
        return frame
```

- [ ] **Step 5: Run the tests — should pass**

Run: `uv run pytest tests/tactile/test_mock.py -v`
Expected: PASS for all 7 tests.

- [ ] **Step 6: Run the full tactile suite**

Run: `uv run pytest tests/tactile/ -v`
Expected: all tests in Tasks 1–5 PASS.

- [ ] **Step 7: Commit**

```bash
git add src/lerobot/tactile/mock/ tests/tactile/test_mock.py
git commit -m "feat(tactile): add MockTactileSensor backend for CI"
```

---

### Task 6: `XelaTactileConfig`

**Files:**

- Create: `src/lerobot/tactile/xela/__init__.py`
- Create: `src/lerobot/tactile/xela/configuration_xela.py`
- Test: `tests/tactile/test_xela_config.py`

- [ ] **Step 1: Write the failing test**

Create `tests/tactile/test_xela_config.py`:

```python
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

from lerobot.tactile.xela.configuration_xela import XelaTactileConfig


def test_default_config():
    cfg = XelaTactileConfig()
    assert cfg.type == "xela"
    assert cfg.host == "127.0.0.1"
    assert cfg.port == 5000
    assert cfg.sensor_id == "1"
    assert cfg.model == "XR1944"
    assert cfg.use_calibrated is False
    assert cfg.tare_on_connect is False
    assert cfg.reconnect_backoff_s == 0.5
    assert cfg.receive_timeout_s == 1.0


def test_shape_inference_from_model():
    """The known-model registry must yield correct flattened shape."""
    cfg = XelaTactileConfig(model="XR1944")
    assert cfg.expected_shape == (48,)  # 16 taxels × 3 axes


def test_unknown_model_raises():
    import pytest
    cfg = XelaTactileConfig(model="NotARealModel")
    with pytest.raises(ValueError, match="NotARealModel"):
        _ = cfg.expected_shape
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `uv run pytest tests/tactile/test_xela_config.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'lerobot.tactile.xela'`.

- [ ] **Step 3: Create the xela package init**

Create `src/lerobot/tactile/xela/__init__.py`:

```python
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

from .configuration_xela import XelaTactileConfig

__all__ = ["XelaTactileConfig"]
# XelaTactileSensor is intentionally not re-exported here; it pulls in `websocket-client`.
# Use `from lerobot.tactile.xela.xela_tactile import XelaTactileSensor` only when needed,
# or rely on `make_tactile_sensors_from_configs` for lazy import.
```

- [ ] **Step 4: Implement `XelaTactileConfig`**

Create `src/lerobot/tactile/xela/configuration_xela.py`:

```python
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

from dataclasses import dataclass

from ..configs import TactileSensorConfig

# Per the XELA Software Manual v1.7.6 (Sensor Layout, p. 36):
#   XR1944 = 4×4 taxels × 3 axes (X, Y, Z)
#   XR1946 = 4×6 taxels × 3 axes
#   XR2244 = magnetic-compensation 4×4 variant
TAXELS_BY_MODEL: dict[str, int] = {
    "XR1944": 16,
    "XR1946": 24,
    "XR2244": 16,
    "XR1922": 16,  # appears in the manual's example messages
}
AXES_PER_TAXEL = 3


@TactileSensorConfig.register_subclass("xela")
@dataclass
class XelaTactileConfig(TactileSensorConfig):
    """Config for an XELA Robotics tactile sensor served by `xela_server` (v1.7.x)."""

    host: str = "127.0.0.1"
    port: int = 5000
    sensor_id: str = "1"
    model: str = "XR1944"
    use_calibrated: bool = False
    tare_on_connect: bool = False
    reconnect_backoff_s: float = 0.5
    receive_timeout_s: float = 1.0

    @property
    def expected_shape(self) -> tuple[int, ...]:
        if self.model not in TAXELS_BY_MODEL:
            raise ValueError(
                f"Unknown XELA model {self.model!r}. "
                f"Known models: {sorted(TAXELS_BY_MODEL)}. "
                "Add an entry to TAXELS_BY_MODEL in configuration_xela.py if your sensor is new."
            )
        return (TAXELS_BY_MODEL[self.model] * AXES_PER_TAXEL,)
```

- [ ] **Step 5: Run the test — should pass**

Run: `uv run pytest tests/tactile/test_xela_config.py -v`
Expected: PASS for all 3 tests.

- [ ] **Step 6: Commit**

```bash
git add src/lerobot/tactile/xela/__init__.py \
        src/lerobot/tactile/xela/configuration_xela.py \
        tests/tactile/test_xela_config.py
git commit -m "feat(tactile): add XelaTactileConfig with model→shape registry"
```

---

### Task 7: XELA frame parser (pure functions, no I/O)

**Files:**

- Create: `src/lerobot/tactile/xela/parser.py`
- Test: `tests/tactile/test_xela_parser.py`

- [ ] **Step 1: Write failing tests against golden-frame fixtures**

Create `tests/tactile/test_xela_parser.py`:

```python
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
        "message": 1, "time": 1.0, "sensors": 1,
        "1": {
            "data": _xr1944_data_string([0] * 48),
            "sensor": "1", "taxels": 16, "model": "XR1944", "calibrated": None,
        },
    }
    with pytest.raises(ValueError, match="model mismatch"):
        parse_frame(json.dumps(msg), sensor_id="1", expected_model="XR2244")


def test_parse_frame_missing_sensor_id_raises():
    msg = {"message": 1, "time": 1.0, "sensors": 1,
           "1": {"data": _xr1944_data_string([0] * 48), "sensor": "1",
                 "taxels": 16, "model": "XR1944", "calibrated": None}}
    with pytest.raises(KeyError, match="sensor_id"):
        parse_frame(json.dumps(msg), sensor_id="2", expected_model="XR1944")


def test_parse_frame_taxel_count_mismatch_raises():
    """If `data` length disagrees with `taxels` field, refuse."""
    bad_data = ",".join(f"{0:04X}" for _ in range(45))  # 45 instead of 48
    msg = {"message": 1, "time": 1.0, "sensors": 1,
           "1": {"data": bad_data, "sensor": "1",
                 "taxels": 16, "model": "XR1944", "calibrated": None}}
    with pytest.raises(ValueError, match="taxel count"):
        parse_frame(json.dumps(msg), sensor_id="1", expected_model="XR1944")
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `uv run pytest tests/tactile/test_xela_parser.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'lerobot.tactile.xela.parser'`.

- [ ] **Step 3: Implement the parser**

Create `src/lerobot/tactile/xela/parser.py`:

```python
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
    calibrated: NDArray[np.float32] | None
    if cal_field is None:
        calibrated = None
    else:
        calibrated = np.asarray(cal_field, dtype=np.float32)

    return ParseResult(
        seq=int(payload["message"]),
        timestamp=float(payload["time"]),
        raw=raw,
        calibrated=calibrated,
    )
```

- [ ] **Step 4: Run the test — should pass**

Run: `uv run pytest tests/tactile/test_xela_parser.py -v`
Expected: PASS for all 8 tests.

- [ ] **Step 5: Commit**

```bash
git add src/lerobot/tactile/xela/parser.py tests/tactile/test_xela_parser.py
git commit -m "feat(tactile): add pure XELA WebSocket frame parser"
```

---

### Task 8: `XelaTactileSensor` (reader thread, last-good-frame, reconnect)

**Files:**

- Create: `src/lerobot/tactile/xela/xela_tactile.py`
- Test: `tests/tactile/test_xela_sensor.py`

- [ ] **Step 1: Write failing tests with mocked WebSocketApp**

Create `tests/tactile/test_xela_sensor.py`:

```python
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
from unittest.mock import MagicMock, patch

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


def test_connect_and_disconnect_lifecycle(patched_ws_app):
    sensor = XelaTactileSensor(XelaTactileConfig(receive_timeout_s=0.5))
    assert sensor.is_connected is False
    sensor.connect()
    # Wait until run_forever has actually been entered (worker thread is up).
    assert patched_ws_app[0].run_forever_called.wait(timeout=2.0)
    assert sensor.is_connected is True
    sensor.disconnect()
    assert sensor.is_connected is False


def test_async_read_returns_first_frame(patched_ws_app):
    sensor = XelaTactileSensor(XelaTactileConfig(receive_timeout_s=2.0))
    sensor.connect()
    fake = patched_ws_app[0]
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
    fake = patched_ws_app[0]
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
    fake = patched_ws_app[0]
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
    patched_ws_app[0].run_forever_called.wait(timeout=2.0)
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
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `uv run pytest tests/tactile/test_xela_sensor.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'lerobot.tactile.xela.xela_tactile'`.

- [ ] **Step 3: Implement `XelaTactileSensor`**

Create `src/lerobot/tactile/xela/xela_tactile.py`:

```python
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

import logging
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
        self._connected = True
        self._thread.start()

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
        url = f"ws://{self.config.host}:{self.config.port}"
        while not self._stop.is_set():
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
            import json
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
        logger.warning("XELA WS closed (status=%s, msg=%s)", status, msg)

    def _on_error(self, ws, error):  # noqa: ANN001
        logger.warning("XELA WS error: %s", error)

    # ----- standalone smoke entrypoint -----

    @staticmethod
    def _cli() -> None:
        """Run as ``python -m lerobot.tactile.xela.xela_tactile --host ...`` for smoke testing."""
        import argparse

        ap = argparse.ArgumentParser(description="XELA tactile sensor smoke test.")
        ap.add_argument("--host", default="127.0.0.1")
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
```

- [ ] **Step 4: Run the test — should pass**

Run: `uv run pytest tests/tactile/test_xela_sensor.py -v`
Expected: PASS for all 7 tests.

If the `pytest.importorskip("websocket")` skips, you missed Task 1 — go back and `uv sync --extra yam`.

- [ ] **Step 5: Run the full tactile suite**

Run: `uv run pytest tests/tactile/ -v`
Expected: every test from Tasks 1–8 PASS.

- [ ] **Step 6: Commit**

```bash
git add src/lerobot/tactile/xela/xela_tactile.py tests/tactile/test_xela_sensor.py
git commit -m "feat(tactile): add XelaTactileSensor WebSocket client backend"
```

---

### Task 9: Wire `tactile_sensors` into `BiYamFollowerConfig`

**Files:**

- Modify: `src/lerobot/robots/bi_yam_follower/config_bi_yam_follower.py`
- Test: `tests/robots/test_bi_yam_tactile.py`

- [ ] **Step 1: Write the failing config test**

Create `tests/robots/test_bi_yam_tactile.py`:

```python
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

from lerobot.robots.bi_yam_follower.config_bi_yam_follower import BiYamFollowerConfig
from lerobot.tactile.mock.mock_tactile import MockTactileConfig


def test_config_default_no_tactile():
    cfg = BiYamFollowerConfig()
    assert cfg.tactile_sensors == {}


def test_config_accepts_tactile_sensor_dict():
    cfg = BiYamFollowerConfig(tactile_sensors={"right_finger_r": MockTactileConfig()})
    assert "right_finger_r" in cfg.tactile_sensors
    assert cfg.tactile_sensors["right_finger_r"].type == "mock"
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `uv run pytest tests/robots/test_bi_yam_tactile.py::test_config_default_no_tactile -v`
Expected: FAIL — `BiYamFollowerConfig` has no `tactile_sensors` attribute.

- [ ] **Step 3: Add the field**

Edit `src/lerobot/robots/bi_yam_follower/config_bi_yam_follower.py`. Add the new import near the top, alongside the camera imports:

```python
from lerobot.tactile.configs import TactileSensorConfig
```

Then add the new field inside the `BiYamFollowerConfig` dataclass, **before** the `cameras` field declaration:

```python
    # Tactile sensors (e.g., XELA pads). Keyed by descriptive name (e.g., "right_finger_r").
    # Each entry is a TactileSensorConfig subclass — see lerobot.tactile.configs.
    tactile_sensors: dict[str, TactileSensorConfig] = field(default_factory=dict)
```

- [ ] **Step 4: Run the test — should pass**

Run: `uv run pytest tests/robots/test_bi_yam_tactile.py -v`
Expected: PASS for both `test_config_*` tests.

- [ ] **Step 5: Commit**

```bash
git add src/lerobot/robots/bi_yam_follower/config_bi_yam_follower.py \
        tests/robots/test_bi_yam_tactile.py
git commit -m "feat(bi_yam): add tactile_sensors field to BiYamFollowerConfig"
```

---

### Task 10: Wire tactile reads into `BiYamFollower.get_observation()`

**Files:**

- Modify: `src/lerobot/robots/bi_yam_follower/bi_yam_follower.py`
- Test: `tests/robots/test_bi_yam_tactile.py` (extend)

- [ ] **Step 1: Add the integration test (failing)**

Append to `tests/robots/test_bi_yam_tactile.py`:

```python
import numpy as np
import pytest
from unittest.mock import MagicMock

from lerobot.robots.bi_yam_follower.bi_yam_follower import BiYamFollower
from lerobot.robots.bi_yam_follower.config_bi_yam_follower import BiYamFollowerConfig
from lerobot.tactile.mock.mock_tactile import MockTactileConfig


@pytest.fixture
def follower_with_mock_tactile(monkeypatch):
    """Construct a BiYamFollower with both arm clients fully mocked + a MockTactileSensor."""
    cfg = BiYamFollowerConfig(tactile_sensors={"right_finger_r": MockTactileConfig()})
    monkeypatch.setattr("lerobot.robots.bi_yam_follower.bi_yam_follower._portal_available", True)

    follower = BiYamFollower(cfg)

    def fake_observations():
        return {"joint_pos": np.zeros(6, dtype=np.float32),
                "gripper_pos": np.zeros(1, dtype=np.float32),
                "joint_eff": np.zeros(6, dtype=np.float32),
                "gripper_eff": np.zeros(1, dtype=np.float32)}

    for arm_attr in ("left_arm", "right_arm"):
        client = MagicMock()
        client.is_connected = True
        client.num_dofs.return_value = 7
        client.get_observations.side_effect = lambda: fake_observations()
        setattr(follower, arm_attr, client)

    follower._left_dofs = 7
    follower._right_dofs = 7
    follower._connected_via_test = True

    # Connect tactile sensors directly (skip the YAM/portal `connect()` path entirely).
    from lerobot.tactile.utils import make_tactile_sensors_from_configs
    follower.tactile_sensors = make_tactile_sensors_from_configs(cfg.tactile_sensors)
    for s in follower.tactile_sensors.values():
        s.connect()

    yield follower

    for s in follower.tactile_sensors.values():
        s.disconnect()


def test_observation_features_include_tactile(follower_with_mock_tactile):
    feats = follower_with_mock_tactile.observation_features
    assert "observation.tactile.right_finger_r" in feats
    assert feats["observation.tactile.right_finger_r"] == (48,)


def test_get_observation_includes_tactile_vector(follower_with_mock_tactile):
    obs = follower_with_mock_tactile.get_observation(include_cameras=False)
    assert "observation.tactile.right_finger_r" in obs
    arr = obs["observation.tactile.right_finger_r"]
    assert isinstance(arr, np.ndarray)
    assert arr.shape == (48,)
    assert arr.dtype == np.float32
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `uv run pytest tests/robots/test_bi_yam_tactile.py -v`
Expected: FAIL — `BiYamFollower` does not yet construct `tactile_sensors` or expose tactile keys.

- [ ] **Step 3: Modify `BiYamFollower.__init__`, `connect`, `disconnect`, `observation_features`, `get_observation`**

Edit `src/lerobot/robots/bi_yam_follower/bi_yam_follower.py`. Add the import alongside the cameras import:

```python
from lerobot.tactile.utils import make_tactile_sensors_from_configs
```

In `__init__` (around line 130, just after `self.cameras = make_cameras_from_configs(config.cameras)`), append:

```python
        # Tactile sensors (XELA, mock, ...). Constructed eagerly; connection happens in connect().
        self.tactile_sensors = make_tactile_sensors_from_configs(config.tactile_sensors)
```

Add a new property next to `_cameras_ft` (around line 199):

```python
    @property
    def _tactile_ft(self) -> dict[str, tuple]:
        """Tactile observation feature shapes, keyed as `observation.tactile.<name>`."""
        return {
            f"observation.tactile.{name}": s.shape
            for name, s in self.tactile_sensors.items()
        }
```

Update `observation_features` to include tactile (around line 207):

```python
    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        """Return observation features including motors, effort/torque, cameras, and tactile."""
        return {**self._motors_ft, **self._effort_ft, **self._cameras_ft, **self._tactile_ft}
```

Update `is_connected` to also check tactile (around line 217):

```python
    @property
    def is_connected(self) -> bool:
        return (
            self.left_arm.is_connected
            and self.right_arm.is_connected
            and all(cam.is_connected for cam in self.cameras.values())
            and all(s.is_connected for s in self.tactile_sensors.values())
        )
```

In `connect()` (after the camera-connect loop, around line 258):

```python
        # Connect tactile sensors. Each backend manages its own background reader thread.
        for name, sensor in self.tactile_sensors.items():
            logger.info("Connecting tactile sensor %r", name)
            sensor.connect()
```

In `disconnect()` (after the cameras-disconnect loop, around line 503):

```python
        for sensor in self.tactile_sensors.values():
            sensor.disconnect()
```

In `get_observation()` (right before the `return obs_dict`, around line 367):

```python
        # Tactile reads — each backend's async_read is non-blocking (last-good-frame on outage).
        for name, sensor in self.tactile_sensors.items():
            obs_dict[f"observation.tactile.{name}"] = sensor.async_read()
```

- [ ] **Step 4: Run the test — should pass**

Run: `uv run pytest tests/robots/test_bi_yam_tactile.py -v`
Expected: PASS for all 4 tests in this file.

- [ ] **Step 5: Run the full tactile + robot test suites**

Run: `uv run pytest tests/tactile/ tests/robots/test_bi_yam_tactile.py tests/robots/test_bi_yam_effort.py -v`
Expected: PASS, no regressions.

- [ ] **Step 6: Commit**

```bash
git add src/lerobot/robots/bi_yam_follower/bi_yam_follower.py \
        tests/robots/test_bi_yam_tactile.py
git commit -m "feat(bi_yam): wire tactile_sensors into observation pipeline"
```

---

### Task 11: `run_xela_server.py` operator wrapper

**Files:**

- Create: `src/lerobot/robots/bi_yam_follower/run_xela_server.py`
- Test: covered manually (script smoke-test only).

- [ ] **Step 1: Create the wrapper script**

Create `src/lerobot/robots/bi_yam_follower/run_xela_server.py`:

```python
#!/usr/bin/env python
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

"""Operator wrapper for launching XELA Server v1.7.x with sensible defaults.

Run this in its own terminal *after* the slcan bus is up:

    sudo slcand -o -s8 -t hw -S 3000000 /dev/ttyUSB0 slcan0
    sudo ifconfig slcan0 up
    python src/lerobot/robots/bi_yam_follower/run_xela_server.py
"""

import argparse
import os
import signal
import subprocess
import sys
from pathlib import Path

DEFAULT_BIN = Path("/etc/xela/xela_server")
DEFAULT_INI = Path("/etc/xela/xServ.ini")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--xela-bin", type=Path, default=DEFAULT_BIN,
                    help="Path to the xela_server AppImage (default: /etc/xela/xela_server).")
    ap.add_argument("--config", type=Path, default=DEFAULT_INI,
                    help="Path to xServ.ini (default: /etc/xela/xServ.ini).")
    ap.add_argument("--ip", default="127.0.0.1",
                    help="Server bind IP. Default 127.0.0.1 to keep the WS local-only.")
    ap.add_argument("--port", type=int, default=5000,
                    help="Server WebSocket port. Default 5000 matches XELA's manual.")
    ap.add_argument("--noros", action="store_true", default=True,
                    help="Pass --noros (default true). Use --noros=false to enable ROS bridge.")
    args = ap.parse_args()

    if not args.xela_bin.exists():
        print(f"error: xela_server binary not found at {args.xela_bin}", file=sys.stderr)
        return 1
    if not args.config.exists():
        print(f"error: xServ.ini not found at {args.config}. "
              f"Run `xela_conf -d socketcan -c slcan0` first.", file=sys.stderr)
        return 1

    cmd = [str(args.xela_bin), "-f", str(args.config),
           "-i", args.ip, "-p", str(args.port)]
    if args.noros:
        cmd.append("--noros")

    print(f"$ {' '.join(cmd)}", flush=True)
    proc = subprocess.Popen(cmd)

    def _forward_sigint(signum, frame):
        proc.send_signal(signum)

    signal.signal(signal.SIGINT, _forward_sigint)
    signal.signal(signal.SIGTERM, _forward_sigint)

    return proc.wait()


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 2: Sanity-check the script's --help works (no XELA hardware needed)**

Run: `uv run python src/lerobot/robots/bi_yam_follower/run_xela_server.py --help`
Expected: prints argparse help with all 4 flags.

- [ ] **Step 3: Commit**

```bash
git add src/lerobot/robots/bi_yam_follower/run_xela_server.py
git commit -m "feat(bi_yam): add run_xela_server.py operator wrapper"
```

---

### Task 12: Documentation

**Files:**

- Create: `src/lerobot/tactile/xela/README.md`
- Modify: `src/lerobot/robots/bi_yam_follower/README.md` (append a new section)

- [ ] **Step 1: Create the XELA backend README**

Create `src/lerobot/tactile/xela/README.md`:

````markdown
# XELA Tactile Backend

A WebSocket client for [XELA Robotics](https://xelarobotics.com/) tactile sensors served
by `xela_server` (v1.7.x). Decodes the JSON wire format documented in §"Data format" of
the XELA Software Manual into a flat `(N,)` `np.float32` vector per read.

## Supported models

| Model    | Layout                       | Channels |
| -------- | ---------------------------- | -------- |
| `XR1944` | 4×4 taxels × 3 axes          | 48       |
| `XR1946` | 4×6 taxels × 3 axes          | 72       |
| `XR2244` | 4×4 (magnetic-comp) × 3 axes | 48       |
| `XR1922` | 4×4 × 3 axes                 | 48       |

Add new models in `configuration_xela.py:TAXELS_BY_MODEL`.

## Wire protocol

Per the manual (p. 37), each WebSocket message is a JSON object:

```json
{
  "message": 786,
  "time": 1619157045.6195483,
  "sensors": 1,
  "1": {
    "data": "FA00,FB00,FC00,...",
    "sensor": "1",
    "taxels": 16,
    "model": "XR1944",
    "calibrated": null
  }
}
```
````

`data` is a comma-separated string of 4-character hex codes (uint16), 3 per taxel
(X, Y, Z). The first message after connect is `{"message": "Welcome", ...}` and is
skipped. Sequence numbers (`message`) must be monotonic; out-of-order frames are dropped.

## Bring-up checklist

```bash
# (1) Activate slcan (one-time per boot — wire to systemd in production)
sudo slcand -o -s8 -t hw -S 3000000 /dev/ttyUSB0 slcan0
sudo ifconfig slcan0 up

# (2) Generate xServ.ini if you haven't yet
xela_conf -d socketcan -c slcan0    # one-shot, writes /etc/xela/xServ.ini

# (3) Start the server (leave running)
python src/lerobot/robots/bi_yam_follower/run_xela_server.py
# or directly: /etc/xela/xela_server -f /etc/xela/xServ.ini --ip 127.0.0.1 -p 5000 --noros

# (4) Smoke-test the stream from Python
python -m lerobot.tactile.xela.xela_tactile --host 127.0.0.1 --port 5000 --sensor-id 1
```

## Failure modes

| Condition                                    | Behaviour                                                                                                                     |
| -------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------- |
| `xela_server` not running at connect         | `XelaTactileSensor.async_read()` raises `TimeoutError` after `receive_timeout_s`.                                             |
| WS disconnect mid-episode                    | Reader auto-reconnects (capped exponential backoff to 2 s); `async_read` returns last-good-frame; one warning per disconnect. |
| Sequence regression                          | Frame dropped silently.                                                                                                       |
| Stale frame (`now − latest_timestamp > 1 s`) | Warning logged each call, data still returned.                                                                                |
| Model mismatch (xServ.ini vs config)         | Frame dropped, warning logged.                                                                                                |

````

- [ ] **Step 2: Append a tactile section to bi_yam_follower/README.md**

Open `src/lerobot/robots/bi_yam_follower/README.md` and append (before the existing "## References" section if present, otherwise at the end):

```markdown
## Tactile Sensor (XELA, optional)

A single XELA XR1944 tactile pad mounted on the right-side fingertip of the right arm's
parallel gripper, served by [`xela_server`](../../tactile/xela/README.md) v1.7.x over
a local WebSocket.

### One-time setup (per boot)

```bash
sudo slcand -o -s8 -t hw -S 3000000 /dev/ttyUSB0 slcan0
sudo ifconfig slcan0 up
xela_conf -d socketcan -c slcan0   # writes /etc/xela/xServ.ini if missing
````

### Per session

```bash
# Terminal A — start XELA server (leave running)
python src/lerobot/robots/bi_yam_follower/run_xela_server.py

# Terminal B — start arm servers (existing pattern, leave running)
python src/lerobot/robots/bi_yam_follower/run_bimanual_yam_server.py

# Terminal C — record with tactile included
lerobot-record \
  --robot.type=bi_yam_follower \
  --robot.tactile_sensors='{
     right_finger_r: {"type":"xela","host":"127.0.0.1","port":5000,
                      "sensor_id":"1","model":"XR1944"}
  }' \
  --teleop.type=bi_yam_leader \
  --dataset.repo_id="${HF_USER}/bimanual-yam-tactile-demo" \
  --dataset.num_episodes=10 \
  --dataset.single_task="Pick and place with tactile feedback" \
  --display_data=true \
  --fps=30
```

### Recorded keys

| Key                                  | Shape   | Dtype     | Notes                                                                              |
| ------------------------------------ | ------- | --------- | ---------------------------------------------------------------------------------- |
| `observation.tactile.right_finger_r` | `(48,)` | `float32` | Raw uint16 magnetic-field readings cast losslessly to float32 (16 taxels × X/Y/Z). |

To enable XCAL-calibrated forces (Newtons) alongside, add `"use_calibrated": true` to
the sensor config; a sibling `observation.tactile.right_finger_r.cal` key appears when
`xela_server` has the `.xcal` files installed.

### Naming convention for adding more sensors

`observation.tactile.<arm>_finger_<side>` where `arm ∈ {left, right}` and
`side ∈ {l, r}` (which jaw of the parallel gripper). Add additional entries to
`--robot.tactile_sensors='{...}'` and the keys appear in the dataset automatically.

````

- [ ] **Step 3: Commit**

```bash
git add src/lerobot/tactile/xela/README.md src/lerobot/robots/bi_yam_follower/README.md
git commit -m "docs(tactile): add XELA backend README and bi_yam tactile section"
````

---

### Task 13: End-to-end suite + lint pass

**Files:** none modified — verification only.

- [ ] **Step 1: Run the full test pyramid touched by this work**

Run: `uv run pytest tests/tactile/ tests/robots/test_bi_yam_tactile.py tests/robots/test_bi_yam_effort.py -v`
Expected: all PASS, no skipped tests other than the deliberate `pytest.importorskip("websocket")` if `websocket-client` is genuinely missing.

- [ ] **Step 2: Run pre-commit on all changes**

Run: `uv run pre-commit run --files $(git diff --name-only main..HEAD)`
Expected: ruff format + lint clean, no `bandit` / typo warnings on the new files.

If `ruff` reports formatting issues, run `uv run ruff format <file>` to fix.

- [ ] **Step 3: Hardware smoke (manual, OPTIONAL — only with real XR1944)**

If the lab's XELA pad is connected and `slcan0` is up:

```bash
# Terminal A
python src/lerobot/robots/bi_yam_follower/run_xela_server.py

# Terminal B
uv run python -m lerobot.tactile.xela.xela_tactile --host 127.0.0.1 --port 5000 --sensor-id 1 --seconds 5
```

Expected: 50 lines of `min=… max=… mean=… ts=…` printed at ~10 Hz. Press the sensor pad with a finger; `max` and `mean` should change.

- [ ] **Step 4: Final commit if any lint fixes were applied**

```bash
git add -u && git commit -m "chore(tactile): lint pass across new tactile subsystem" || echo "Nothing to commit."
```

- [ ] **Step 5: Summary check against the spec**

Open the spec [docs/superpowers/specs/2026-04-29-xela-tactile-integration-design.md](../specs/2026-04-29-xela-tactile-integration-design.md) and confirm:

- [ ] Section 1 (Architecture) — file tree implemented as listed
- [ ] Section 2 (Public API) — `TactileSensor` ABC + `XelaTactileConfig` shipped
- [ ] Section 3 (Data flow) — reader thread + last-good-frame pattern shipped
- [ ] Section 4 (Observation schema) — `observation.tactile.right_finger_r` (48,) float32 verified in `test_get_observation_includes_tactile_vector`
- [ ] Section 5 (Parser) — pure functions in `parser.py` covered by `test_xela_parser.py`
- [ ] Section 6 (Bring-up runbook) — present in both READMEs
- [ ] Section 7 (Failure modes) — soft-fail / reconnect / stale-warn all in `xela_tactile.py`
- [ ] Section 8 (Testing) — unit + integration tests present
- [ ] Section 9 (Out of scope) — no auto-spawn, no ROS, no Rerun heatmap (correct)
- [ ] Section 10 (File-level diff) — every file in the table exists with the expected purpose

---

## Self-Review

After writing this plan, I checked it against the spec:

**1. Spec coverage:** Every numbered section in the spec is mapped to a task — see Task 13's checklist. The full file-level diff table from spec Section 10 is realised by Tasks 1–12.

**2. Placeholder scan:** No "TBD"/"TODO"/"add appropriate error handling" markers in any task. Every code step contains the actual code.

**3. Type consistency:**

- `TactileSensor.shape` → `tuple[int, ...]` (used identically in `MockTactileSensor`, `XelaTactileSensor`, and `BiYamFollower._tactile_ft`).
- `TactileSensor.dtype` → `np.dtype` (consistent in both backends).
- `TactileSensor.async_read()` → `NDArray[np.float32]` (consistent across mock + xela + integration test assertions).
- `XelaTactileConfig.expected_shape` → `tuple[int, ...]` (used by `XelaTactileSensor.__init__` to seed `self._shape`).
- `ParseResult` fields (`seq: int`, `timestamp: float`, `raw: NDArray[np.float32]`, `calibrated: NDArray[np.float32] | None`) used identically in parser tests and the sensor's `_on_message`.
- Observation key `observation.tactile.right_finger_r` is consistent across `_tactile_ft`, `get_observation`, the integration test assertions, and the README example.

No inconsistencies found.

---

## Post-implementation amendments (2026-04-30)

The plan's task code blocks above represent the plan _as written_ before execution. Three corrections were made during execution after smoke-testing against real XR1944 hardware revealed limitations of XELA Server v1.7.6 build 158509. The shipped code (and the `tactile` branch) reflects these corrections; the originals are kept here as historical record.

### A. `XelaTactileConfig.host` default: `"127.0.0.1"` → `"auto"`

**Why:** the `xela_server` AppImage v1.7.6 build 158509 silently ignores the `--ip` flag and always binds to the host's primary NIC IP (e.g. `192.168.x.x`). Pinning the client to `127.0.0.1` would never connect on this machine.

**Fix:** default `host` to `"auto"`, resolved at connect time by `_resolve_host()` using a UDP-route trick (the same heuristic XELA uses internally). Existing IPs/hostnames pass through unchanged.

**Files touched:** `src/lerobot/tactile/xela/configuration_xela.py`, `src/lerobot/tactile/xela/xela_tactile.py`, `tests/tactile/test_xela_config.py`, `tests/tactile/test_xela_sensor.py` (added `_wait_for_first` helper to handle the new resolve-step race), commits `c030ab03` and `d4714064`.

### B. Drop `--ip` from `run_xela_server.py` invocations

**Why:** same as above. Passing the flag was misleading (suggested it worked) and Task 11's wrapper used `-i` (the short form), which xela_server's AppImage doesn't recognise either. The only effect of either was a silent no-op.

**Fix:** `run_xela_server.py` now omits `--ip` unless the operator forces one explicitly. README/spec invocations updated to drop the flag and explain the quirk.

**Files touched:** `src/lerobot/robots/bi_yam_follower/run_xela_server.py`, `src/lerobot/tactile/xela/README.md`, `src/lerobot/robots/bi_yam_follower/README.md`, commit `c030ab03`.

### C. Graceful WS-close demoted to INFO

**Why:** the smoke test surfaced a noisy `WARNING: XELA WS closed (status=None, msg=None)` on every clean `disconnect()`. Warnings should be reserved for unexpected drops (the reconnect-worthy case).

**Fix:** `_on_close` keys off `self._stop.is_set()` — INFO when we asked for the close, WARNING otherwise. Two regression tests added (`test_graceful_close_logs_info_not_warning`, `test_unexpected_close_logs_warning`).

**Files touched:** `src/lerobot/tactile/xela/xela_tactile.py`, `tests/tactile/test_xela_sensor.py`, commit `275e6298`.

### D. Inspection utility added (out of original plan scope)

**Why:** Task 13 left "how to examine recorded data" implicit. After hardware validation we packaged the demo snippets into a runnable utility so future contributors don't have to re-derive them.

**Files touched:** `examples/tactile/inspect_tactile_dataset.py`, `examples/tactile/README.md` (new), `src/lerobot/tactile/xela/README.md` and `src/lerobot/robots/bi_yam_follower/README.md` updated with cross-references.

### Final test count: 44/44 (was 40/40 in the original plan)

Two `_resolve_host` tests + two WS-close logging assertion tests landed in amendments A and C.
