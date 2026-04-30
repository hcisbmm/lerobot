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

# Import backend configs so their @TactileSensorConfig.register_subclass decorators
# run before draccus parses CLI args. Without this, `--robot.tactile_sensors='{...:
# {type: xela, ...}}'` fails with "Couldn't find a choice class for 'xela'".
# Sensor classes (which may pull in optional deps) stay lazy — see utils.make_tactile_sensors_from_configs.
from .mock.mock_tactile import MockTactileConfig  # noqa: F401
from .xela.configuration_xela import XelaTactileConfig  # noqa: F401

__all__ = ["TactileSensor", "TactileSensorConfig"]
