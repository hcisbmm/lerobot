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
    """Config for an XELA Robotics tactile sensor served by `xela_server` (v1.7.x).

    Note on `host`: XELA Server v1.7.6 build 158509 silently ignores the `--ip`
    flag and always binds to the host's primary NIC IP (e.g., 192.168.x.x). Pass
    `host="auto"` to let `XelaTactileSensor` resolve that IP at connect time
    using the same heuristic XELA uses (the route to a public IP). Override with
    an explicit IP/hostname if you know the bind address.
    """

    host: str = "auto"
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
