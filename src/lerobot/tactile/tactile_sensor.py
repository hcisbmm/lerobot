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
