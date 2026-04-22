#!/usr/bin/env python

# Copyright 2025 Nvidia and The HuggingFace Inc. team. All rights reserved.
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

from .configuration_groot import GrootConfig

try:
    from .modeling_groot import GrootPolicy
    from .processor_groot import make_groot_pre_post_processors
except ImportError:
    # Groot pulls in `diffusers` (optional); if it's not installed, the config is
    # still importable but GrootPolicy / make_groot_pre_post_processors are not.
    # The policy factory fails loudly at instantiation time if users actually pick groot.
    pass

__all__ = ["GrootConfig", "GrootPolicy", "make_groot_pre_post_processors"]
