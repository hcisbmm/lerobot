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
