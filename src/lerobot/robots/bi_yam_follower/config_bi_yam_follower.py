
#!/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
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

from dataclasses import dataclass, field

from lerobot.cameras.configs import CameraConfig, Cv2Rotation
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig

from ..config import RobotConfig


@RobotConfig.register_subclass("bi_yam_follower")
@dataclass
class BiYamFollowerConfig(RobotConfig):
    # Server ports for left and right arm followers
    # These should match the ports in the bimanual_lead_follower.py script
    # Default: 1235 for left arm, 1234 for right arm
    left_arm_port: int = 1235
    right_arm_port: int = 1234

    # Server host (usually localhost for local setup)
    server_host: str = "localhost"

    # Optional: Maximum relative target for safety
    left_arm_max_relative_target: float | dict[str, float] | None = None
    right_arm_max_relative_target: float | dict[str, float] | None = None

    # Palm camera toggle and settings
    # Enable with --robot.use_palm_camera=true
    # Use palm_camera_fourcc="MJPG" and palm_camera_fps=30 for higher frame rate
    use_palm_camera: bool = False
    palm_camera_fps: int = 30
    palm_camera_fourcc: str | None = "MJPG"

    # Cameras (shared between both arms)
    # When use_palm_camera=true, palm cameras are merged with any explicitly provided cameras
    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    def __post_init__(self):
        if self.use_palm_camera:
            palm_cameras: dict[str, CameraConfig] = {
                "left_palm": OpenCVCameraConfig(
                    index_or_path="/dev/v4l/by-path/pci-0000:00:14.0-usb-0:6.2.4:1.0-video-index0",
                    fps=self.palm_camera_fps,
                    width=480,
                    height=640,
                    rotation=Cv2Rotation.ROTATE_90,
                    warmup_s=3,
                    fourcc=self.palm_camera_fourcc,
                ),
                "right_palm": OpenCVCameraConfig(
                    index_or_path="/dev/v4l/by-path/pci-0000:00:14.0-usb-0:6.3.4:1.0-video-index0",
                    fps=self.palm_camera_fps,
                    width=480,
                    height=640,
                    rotation=Cv2Rotation.ROTATE_270,
                    warmup_s=3,
                    fourcc=self.palm_camera_fourcc,
                ),
            }
            # Merge: palm cameras as base, explicit --robot.cameras overrides on collision
            palm_cameras.update(self.cameras)
            self.cameras = palm_cameras
        super().__post_init__()
