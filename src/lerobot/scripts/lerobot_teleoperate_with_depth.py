#!/usr/bin/env python

# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
"""
Teleoperate with live depth stream display (no dataset).

Thin wrapper around `lerobot.scripts.lerobot_teleoperate.teleoperate` that
attaches depth-stream capture to the robot in-place — any camera listed in
`--depth_cams` emits a `{cam}_depth` observation key (uint16 mm) alongside
its regular color frame. The depth observations flow through the Rerun
display pipeline when `--display_data=true`, so you can sanity-check the
depth stream without starting a recording session.

Use this for: validating RealSense depth alignment, bilateral control tuning,
quick-look before committing to a full `lerobot-record-with-depth` run.

Example:

    lerobot-teleoperate-with-depth \
        --robot.type=bi_yam_follower \
        --robot.cameras='{top: {"type": "intelrealsense", "serial_number_or_name": "141722076304", "width": 640, "height": 480, "fps": 30, "use_depth": true}}' \
        --teleop.type=bi_yam_leader \
        --display_data=true \
        --depth_cams='[top]'
"""

import logging
from contextlib import contextmanager
from dataclasses import dataclass, field
from pprint import pformat

from lerobot.configs import parser
from lerobot.scripts.lerobot_teleoperate import TeleoperateConfig, teleoperate

# Reuse the depth-attach helper from the record wrapper so there's one source
# of truth for how depth streams are wired into a Robot instance.
from lerobot.scripts.lerobot_record_with_depth import _attach_depth_capability
from lerobot.utils.utils import init_logging

logger = logging.getLogger(__name__)

# Call the unwrapped teleoperate so parser.wrap doesn't re-parse sys.argv
# under the parent TeleoperateConfig (which lacks --depth_cams).
_teleoperate_impl = getattr(teleoperate, "__wrapped__", teleoperate)


@contextmanager
def _patch_robot_factory(depth_cams: list[str]):
    """Wrap `make_robot_from_config` inside the teleoperate module so the
    robot returned by the pipeline emits depth observations.
    """
    from lerobot.scripts import lerobot_teleoperate as teleop_mod

    original = teleop_mod.make_robot_from_config

    def patched(cfg):
        robot = original(cfg)
        return _attach_depth_capability(robot, depth_cams)

    teleop_mod.make_robot_from_config = patched
    try:
        yield
    finally:
        teleop_mod.make_robot_from_config = original


@dataclass
class DepthTeleoperateConfig(TeleoperateConfig):
    # Cameras to ALSO capture depth from. Each must be configured with
    # use_depth=True (e.g. RealSenseCameraConfig).
    depth_cams: list[str] = field(default_factory=list)


@parser.wrap()
def main(cfg: DepthTeleoperateConfig):
    init_logging()
    logger.info("DepthTeleoperateConfig:\n%s", pformat(cfg))

    if not cfg.depth_cams:
        logger.warning(
            "--depth_cams is empty; falling through to plain lerobot-teleoperate behavior."
        )
        return _teleoperate_impl(cfg)

    with _patch_robot_factory(cfg.depth_cams):
        return _teleoperate_impl(cfg)


if __name__ == "__main__":
    main()
