# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
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

import numbers
import os
import time
from collections import deque

import numpy as np

from lerobot.types import RobotAction, RobotObservation

from .constants import ACTION, ACTION_PREFIX, OBS_PREFIX, OBS_STR
from .import_utils import require_package


def init_rerun(
    session_name: str = "lerobot_control_loop", ip: str | None = None, port: int | None = None
) -> None:
    """
    Initializes the Rerun SDK for visualizing the control loop.

    Args:
        session_name: Name of the Rerun session.
        ip: Optional IP for connecting to a Rerun server.
        port: Optional port for connecting to a Rerun server.
    """

    require_package("rerun-sdk", extra="viz", import_name="rerun")
    import rerun as rr

    batch_size = os.getenv("RERUN_FLUSH_NUM_BYTES", "8000")
    os.environ["RERUN_FLUSH_NUM_BYTES"] = batch_size
    rr.init(session_name)
    memory_limit = os.getenv("LEROBOT_RERUN_MEMORY_LIMIT", "10%")
    if ip and port:
        rr.connect_grpc(url=f"rerun+http://{ip}:{port}/proxy")
    else:
        rr.spawn(memory_limit=memory_limit)


def shutdown_rerun() -> None:
    """Shuts down the Rerun SDK gracefully."""

    require_package("rerun-sdk", extra="viz", import_name="rerun")
    import rerun as rr

    rr.rerun_shutdown()


def _is_scalar(x):
    return isinstance(x, (float | numbers.Real | np.integer | np.floating)) or (
        isinstance(x, np.ndarray) and x.ndim == 0
    )


def log_rerun_data(
    observation: RobotObservation | None = None,
    action: RobotAction | None = None,
    compress_images: bool = False,
) -> None:
    """
    Logs observation and action data to Rerun for real-time visualization.

    This function iterates through the provided observation and action dictionaries and sends their contents
    to the Rerun viewer. It handles different data types appropriately:
    - Scalars values (floats, ints) are logged as `rr.Scalars`.
    - 3D NumPy arrays that resemble images (e.g., with 1, 3, or 4 channels first) are transposed
      from CHW to HWC format, (optionally) compressed to JPEG and logged as `rr.Image` or `rr.EncodedImage`.
    - 1D NumPy arrays are logged as a series of individual scalars, with each element indexed.
    - Other multi-dimensional arrays are flattened and logged as individual scalars.

    Keys are automatically namespaced with "observation." or "action." if not already present.

    Args:
        observation: An optional dictionary containing observation data to log.
        action: An optional dictionary containing action data to log.
        compress_images: Whether to compress images before logging to save bandwidth & memory in exchange for cpu and quality.
    """

    require_package("rerun-sdk", extra="viz", import_name="rerun")
    import rerun as rr

    if observation:
        for k, v in observation.items():
            if v is None:
                continue
            key = k if str(k).startswith(OBS_PREFIX) else f"{OBS_STR}.{k}"

            if _is_scalar(v):
                rr.log(key, rr.Scalars(float(v)))
            elif isinstance(v, np.ndarray):
                arr = v
                # Convert CHW -> HWC when needed
                if arr.ndim == 3 and arr.shape[0] in (1, 3, 4) and arr.shape[-1] not in (1, 3, 4):
                    arr = np.transpose(arr, (1, 2, 0))
                if arr.ndim == 1:
                    for i, vi in enumerate(arr):
                        rr.log(f"{key}_{i}", rr.Scalars(float(vi)))
                else:
                    img_entity = rr.Image(arr).compress() if compress_images else rr.Image(arr)
                    rr.log(key, entity=img_entity, static=True)

    if action:
        for k, v in action.items():
            if v is None:
                continue
            key = k if str(k).startswith(ACTION_PREFIX) else f"{ACTION}.{k}"

            if _is_scalar(v):
                rr.log(key, rr.Scalars(float(v)))
            elif isinstance(v, np.ndarray):
                if v.ndim == 1:
                    for i, vi in enumerate(v):
                        rr.log(f"{key}_{i}", rr.Scalars(float(vi)))
                else:
                    # Fall back to flattening higher-dimensional arrays
                    flat = v.flatten()
                    for i, vi in enumerate(flat):
                        rr.log(f"{key}_{i}", rr.Scalars(float(vi)))


class TorqueVisualizer:
    """Real-time rolling torque plot for robot arms using matplotlib.

    Extracts keys ending with '.eff' from observation dicts and displays
    them as a live rolling plot. Designed to run inside an existing control
    loop (teleop / record) without blocking it.
    """

    JOINT_LABELS = ["J1", "J2", "J3", "J4", "J5", "J6", "Gripper"]
    ARM_COLORS = {"left": "#3cb44b", "right": "#e6194b"}
    JOINT_LINESTYLES = ["-", "--", "-.", ":", (0, (3, 1, 1, 1)), (0, (5, 2)), (0, (1, 1))]

    def __init__(self, window_seconds: float = 10.0, ee_only: bool = False):
        self.window_seconds = window_seconds
        self.ee_only = ee_only
        self._initialized = False
        self._t_start = time.time()
        self._max_samples = 2000
        self._time_buf: deque = deque(maxlen=self._max_samples)
        self._torque_bufs: dict[str, deque] = {}
        self._lines: dict = {}
        self._value_texts: dict = {}
        self._fig = None
        self._ax = None
        self._frame_count = 0

    def _init_plot(self, eff_keys: list[str]) -> None:
        """Initialize the matplotlib figure with discovered effort keys."""
        import matplotlib.pyplot as plt

        plt.ion()
        self._fig, self._ax = plt.subplots(figsize=(14, 7))

        sorted_keys = sorted(eff_keys)
        for idx, key in enumerate(sorted_keys):
            # Parse arm side and joint index from key like "left_joint_0.eff" or "right_gripper.eff"
            parts = key.replace(".eff", "")
            if parts.startswith("left_"):
                arm = "left"
                remainder = parts[len("left_"):]
            elif parts.startswith("right_"):
                arm = "right"
                remainder = parts[len("right_"):]
            else:
                arm = "unknown"
                remainder = parts

            if remainder == "gripper":
                joint_idx = 6
                joint_label = "Gripper"
            elif remainder.startswith("joint_"):
                joint_idx = int(remainder.split("_")[1])
                joint_label = self.JOINT_LABELS[joint_idx] if joint_idx < len(self.JOINT_LABELS) else f"J{joint_idx}"
            else:
                joint_idx = idx
                joint_label = remainder

            color = self.ARM_COLORS.get(arm, "#999999")
            ls = self.JOINT_LINESTYLES[joint_idx % len(self.JOINT_LINESTYLES)]
            label = f"{arm.capitalize()} {joint_label}"

            (line,) = self._ax.plot([], [], label=label, color=color, linestyle=ls, linewidth=1.5)
            self._lines[key] = line
            self._torque_bufs[key] = deque(maxlen=self._max_samples)

            txt = self._ax.text(
                1.01,
                1.0 - idx * (1.0 / max(len(sorted_keys), 1)),
                "",
                transform=self._ax.transAxes,
                fontsize=7,
                color=color,
                verticalalignment="top",
                fontfamily="monospace",
            )
            self._value_texts[key] = txt

        self._ax.set_xlabel("Time (s)")
        self._ax.set_ylabel("Torque (Nm)")
        title = "End-Effector Torques (Followers)" if self.ee_only else "Real-Time Joint Torques (Follower Arms)"
        self._ax.set_title(title)
        self._ax.legend(loc="upper right", fontsize=7, ncol=2)
        self._ax.grid(True, alpha=0.3)
        self._fig.tight_layout()
        self._fig.subplots_adjust(right=0.78)
        self._fig.canvas.draw()
        self._fig.canvas.flush_events()
        self._initialized = True

    def _filter_eff_keys(self, obs: dict) -> dict[str, float]:
        """Extract .eff keys from observation, optionally filtering to EE only."""
        eff_data = {}
        for k, v in obs.items():
            if not k.endswith(".eff"):
                continue
            if self.ee_only and "gripper" not in k:
                continue
            eff_data[k] = float(v)
        return eff_data

    def update(self, obs: dict) -> None:
        """Buffer new torque data and periodically redraw the plot."""
        eff_data = self._filter_eff_keys(obs)
        if not eff_data:
            return

        if not self._initialized:
            self._init_plot(list(eff_data.keys()))

        t_now = time.time() - self._t_start
        self._time_buf.append(t_now)

        for key, val in eff_data.items():
            if key in self._torque_bufs:
                self._torque_bufs[key].append(val)

        # Redraw at ~10 Hz (every 6 frames at 60fps)
        self._frame_count += 1
        if self._frame_count % 6 == 0:
            self._redraw()

    def _redraw(self) -> None:
        """Update matplotlib lines and axes."""
        if self._fig is None:
            return

        t_arr = np.array(self._time_buf)
        all_vals = []

        for key, line in self._lines.items():
            buf = self._torque_bufs[key]
            if len(buf) > 0:
                y_arr = np.array(buf)
                line.set_data(t_arr[: len(y_arr)], y_arr)
                all_vals.append(y_arr)
                self._value_texts[key].set_text(f"{line.get_label()}: {y_arr[-1]:+.3f}")

        if len(t_arr) > 1:
            self._ax.set_xlim(max(0, t_arr[-1] - self.window_seconds), t_arr[-1] + 0.5)
            if all_vals:
                concat = np.concatenate(all_vals)
                y_min, y_max = np.min(concat), np.max(concat)
                y_pad = max(0.5, (y_max - y_min) * 0.1)
                self._ax.set_ylim(y_min - y_pad, y_max + y_pad)

        self._fig.canvas.draw_idle()
        self._fig.canvas.flush_events()

    def close(self) -> None:
        """Close the plot window."""
        if self._fig is not None:
            import matplotlib.pyplot as plt

            plt.close(self._fig)
            self._fig = None
