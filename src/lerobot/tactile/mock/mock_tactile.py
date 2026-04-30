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
