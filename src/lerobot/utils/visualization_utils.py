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


# ---------------------------------------------------------------------------
# Shared helpers for bimanual YAM (.eff / .pos) observation parsing
# Used by TorqueVisualizer (2D matplotlib) and MujocoTorqueVisualizer (3D).
# ---------------------------------------------------------------------------

BI_YAM_JOINT_ORDER: list[str] = [
    "left_joint_0", "left_joint_1", "left_joint_2",
    "left_joint_3", "left_joint_4", "left_joint_5", "left_gripper",
    "right_joint_0", "right_joint_1", "right_joint_2",
    "right_joint_3", "right_joint_4", "right_joint_5", "right_gripper",
]  # canonical 14-DoF order (matches bi_yam_follower observation schema)


def parse_effort_key(key: str) -> tuple[str, str] | None:
    """Parse a bi_yam effort/position key into (arm_side, part_name).

    Returns ('left', 'joint_0') for 'left_joint_0.eff', ('right', 'gripper')
    for 'right_gripper.eff', and None for anything that doesn't fit the pattern.
    Works for both '.eff' and '.pos' suffixes.
    """
    if "." not in key:
        return None
    base, _, _ = key.rpartition(".")
    for arm in ("left", "right"):
        prefix = f"{arm}_"
        if base.startswith(prefix):
            part = base[len(prefix):]
            if part == "gripper":
                return arm, part
            if part.startswith("joint_") and part[len("joint_"):].isdigit():
                return arm, part
    return None


def is_gripper_key(key: str) -> bool:
    """True for gripper-related effort/position keys (used for EE-only filtering)."""
    return "gripper" in key


def extract_ordered(
    obs: dict,
    suffix: str,
    joint_order: list[str] = BI_YAM_JOINT_ORDER,
) -> np.ndarray:
    """Pull scalars from obs into a fixed-length float64 ndarray aligned to joint_order.

    Reads obs[f'{name}.{suffix}'] for each name; missing keys zero-fill. Used by both
    visualizers to get stable arrays for .pos (MuJoCo qpos mirroring) and .eff (arrow
    rendering) regardless of dict iteration order or missing hardware values.
    """
    return np.array(
        [float(obs.get(f"{name}.{suffix}", 0.0)) for name in joint_order],
        dtype=np.float64,
    )


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
            parsed = parse_effort_key(key)
            if parsed is None:
                arm, remainder = "unknown", key
            else:
                arm, remainder = parsed

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
            if self.ee_only and not is_gripper_key(k):
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


# ---------------------------------------------------------------------------
# MujocoTorqueVisualizer — 3D live torque + EE-force visualization
# ---------------------------------------------------------------------------


# Map each bi_yam observation key (14 DoFs) to its MuJoCo joint name in the
# bimanual scene composed by mujoco_scene_builder. Gripper obs maps to joint7;
# joint8 is coupled via equality and mirrored manually during state sync.
_OBS_TO_MJ_JOINT: dict[str, str] = {
    "left_joint_0": "left_joint1",
    "left_joint_1": "left_joint2",
    "left_joint_2": "left_joint3",
    "left_joint_3": "left_joint4",
    "left_joint_4": "left_joint5",
    "left_joint_5": "left_joint6",
    "left_gripper":  "left_joint7",
    "right_joint_0": "right_joint1",
    "right_joint_1": "right_joint2",
    "right_joint_2": "right_joint3",
    "right_joint_3": "right_joint4",
    "right_joint_4": "right_joint5",
    "right_joint_5": "right_joint6",
    "right_gripper": "right_joint7",
}
_GRIPPER_MIRRORS: list[tuple[str, str]] = [
    ("left_joint7", "left_joint8"),
    ("right_joint7", "right_joint8"),
]
_LEFT_ARM_OBS = [f"left_joint_{i}" for i in range(6)]
_RIGHT_ARM_OBS = [f"right_joint_{i}" for i in range(6)]


class MujocoTorqueVisualizer:
    """Live 3D torque + EE-force visualization for the bi_yam_follower robot.

    Mirrors the robot's joint positions into a bimanual YAM MuJoCo scene,
    draws an arrow along each joint axis sized and colored by the measured
    torque (from the robot's .eff observation keys), and optionally overlays
    an estimated end-effector force vector at each gripper site using the
    Jacobian pseudoinverse relation ``F_ee = pinv(J^T)(τ_meas − g(q))``.

    Lifecycle mirrors :class:`TorqueVisualizer`: construct once, call
    :meth:`update` every control-loop iteration with the full observation
    dict, and :meth:`close` on shutdown. The viewer sync is throttled to
    ``target_fps`` so a 60 Hz teleop loop doesn't stall on rendering.

    Physics caveats:
    - ``.eff`` is N·m direct from the motor firmware (DM MIT-mode CAN
      feedback). No K_t conversion is done in this code. The estimate
      captures motor-side friction but not cable/joint-side friction.
    - The EE force is a quasi-static estimate. Unmodeled dynamics
      (M·q̈ + C·q̇) and friction leak into the result; at teleop speeds
      this is small but not zero. Gravity compensation uses MuJoCo's
      inverse-dynamics on the supplied MJCF — if the real arm carries a
      custom end-effector mass not reflected in the model, the estimate
      is off by that amount.
    - Near arm singularities, the pseudoinverse gain blows up. Magnitudes
      are clamped to ``max_ee_force`` to keep the overlay readable.
    """

    _POS_COLOR = np.array([0.85, 0.15, 0.15, 0.9], dtype=np.float32)
    _NEG_COLOR = np.array([0.15, 0.25, 0.85, 0.9], dtype=np.float32)
    _FORCE_COLOR = np.array([0.10, 0.75, 0.25, 0.9], dtype=np.float32)

    def __init__(
        self,
        gripper_type: str = "linear_4310",
        arrow_scale: float = 0.05,
        arrow_width: float = 0.005,
        force_arrow_scale: float = 0.01,
        force_arrow_width: float = 0.008,
        show_ee_force: bool = True,
        compensate_gravity: bool = True,
        max_ee_force: float = 50.0,
        left_base_offset: tuple[float, float, float] = (0.0, 0.25, 0.0),
        right_base_offset: tuple[float, float, float] = (0.0, -0.25, 0.0),
        target_fps: float = 30.0,
    ) -> None:
        require_package("mujoco", extra="yam")

        import mujoco
        import mujoco.viewer

        from lerobot.utils.mujoco_scene_builder import build_bimanual_yam_scene

        self.arrow_scale = float(arrow_scale)
        self.arrow_width = float(arrow_width)
        self.force_arrow_scale = float(force_arrow_scale)
        self.force_arrow_width = float(force_arrow_width)
        self.show_ee_force = bool(show_ee_force)
        self.compensate_gravity = bool(compensate_gravity)
        self.max_ee_force = float(max_ee_force)
        self.target_fps = float(target_fps)

        xml = build_bimanual_yam_scene(
            gripper_type=gripper_type,
            left_base_offset=left_base_offset,
            right_base_offset=right_base_offset,
        )
        self._mj = mujoco
        self._model = mujoco.MjModel.from_xml_string(xml)
        self._data = mujoco.MjData(self._model)

        self._joint_ids: dict[str, int] = {
            obs_key: self._model.joint(mj_name).id
            for obs_key, mj_name in _OBS_TO_MJ_JOINT.items()
        }
        self._joint_qposadr: dict[str, int] = {
            obs_key: int(self._model.jnt_qposadr[self._joint_ids[obs_key]])
            for obs_key in _OBS_TO_MJ_JOINT
        }
        self._mirror_qposadr: list[tuple[int, int]] = [
            (
                int(self._model.jnt_qposadr[self._model.joint(master).id]),
                int(self._model.jnt_qposadr[self._model.joint(slave).id]),
            )
            for master, slave in _GRIPPER_MIRRORS
        ]
        # Gripper obs (e.g. left_gripper.pos) is normalized [0,1] from run_yam_server,
        # but the MJCF slide joint is in meters. Scale by the joint's physical stroke so
        # fingers open the correct distance instead of flying meters apart.
        gripper_masters = {master for master, _ in _GRIPPER_MIRRORS}
        self._gripper_scale: dict[str, float] = {}
        for obs_key, mj_name in _OBS_TO_MJ_JOINT.items():
            if mj_name not in gripper_masters:
                continue
            lo, hi = self._model.jnt_range[self._model.joint(mj_name).id]
            stroke = float(hi - lo)
            if stroke > 0.0:
                self._gripper_scale[obs_key] = stroke
        self._left_arm_dofadr = np.array(
            [int(self._model.jnt_dofadr[self._model.joint(f"left_joint{i}").id]) for i in range(1, 7)],
            dtype=np.int32,
        )
        self._right_arm_dofadr = np.array(
            [int(self._model.jnt_dofadr[self._model.joint(f"right_joint{i}").id]) for i in range(1, 7)],
            dtype=np.int32,
        )
        self._left_grasp_site_id = int(self._model.site("left_grasp_site").id)
        self._right_grasp_site_id = int(self._model.site("right_grasp_site").id)

        self._viewer = mujoco.viewer.launch_passive(self._model, self._data)
        self._closed = False
        self._last_redraw = 0.0

    def update(self, obs: dict) -> None:
        """Mirror joint positions into MuJoCo and redraw arrows. Throttled to ``target_fps``."""
        if self._closed or obs is None:
            return

        now = time.monotonic()
        if now - self._last_redraw < 1.0 / self.target_fps:
            return
        self._last_redraw = now

        self._mirror_qpos(obs)
        self._mj.mj_forward(self._model, self._data)

        torques = extract_ordered(obs, "eff")
        self._update_user_scn(obs, torques)

        self._viewer.sync()

    def close(self) -> None:
        """Close the viewer. Idempotent."""
        if self._closed:
            return
        self._closed = True
        try:
            self._viewer.close()
        except Exception:
            pass

    def _mirror_qpos(self, obs: dict) -> None:
        for obs_key, qposadr in self._joint_qposadr.items():
            val = obs.get(f"{obs_key}.pos")
            if val is None:
                continue
            scale = self._gripper_scale.get(obs_key)
            if scale is not None:
                val = float(val) * scale
            self._data.qpos[qposadr] = float(val)
        for master_adr, slave_adr in self._mirror_qposadr:
            self._data.qpos[slave_adr] = self._data.qpos[master_adr]

    def _update_user_scn(self, obs: dict, torques: np.ndarray) -> None:
        scn = self._viewer.user_scn
        scn.ngeom = 0

        for i, obs_key in enumerate(BI_YAM_JOINT_ORDER):
            tau = float(torques[i])
            if abs(tau) < 1e-6:
                continue
            jnt_id = self._joint_ids[obs_key]
            anchor = np.asarray(self._data.xanchor[jnt_id], dtype=np.float64)
            axis = np.asarray(self._data.xaxis[jnt_id], dtype=np.float64)
            length = self.arrow_scale * abs(tau)
            direction = axis * (1.0 if tau > 0 else -1.0)
            to_pt = anchor + direction * length
            color = self._POS_COLOR if tau > 0 else self._NEG_COLOR
            self._add_arrow(scn, anchor, to_pt, self.arrow_width, color)

        if self.show_ee_force:
            g = self._compute_gravity() if self.compensate_gravity else None
            for side, arm_keys, site_id, dofadr in (
                ("left", _LEFT_ARM_OBS, self._left_grasp_site_id, self._left_arm_dofadr),
                ("right", _RIGHT_ARM_OBS, self._right_grasp_site_id, self._right_arm_dofadr),
            ):
                F = self._compute_ee_force(arm_keys, dofadr, site_id, obs, g)
                if F is None:
                    continue
                site_pos = np.asarray(self._data.site_xpos[site_id], dtype=np.float64)
                mag = float(np.linalg.norm(F))
                if mag < 1e-6:
                    continue
                mag_clamped = min(mag, self.max_ee_force)
                direction = F / mag
                to_pt = site_pos + direction * (self.force_arrow_scale * mag_clamped)
                self._add_arrow(scn, site_pos, to_pt, self.force_arrow_width, self._FORCE_COLOR)

    def _add_arrow(self, scn, from_pt, to_pt, width, rgba):
        if scn.ngeom >= scn.maxgeom:
            return
        geom = scn.geoms[scn.ngeom]
        self._mj.mjv_initGeom(
            geom,
            type=self._mj.mjtGeom.mjGEOM_ARROW,
            size=np.zeros(3, dtype=np.float64),
            pos=np.zeros(3, dtype=np.float64),
            mat=np.eye(3, dtype=np.float64).flatten(),
            rgba=rgba,
        )
        self._mj.mjv_connector(
            geom,
            type=self._mj.mjtGeom.mjGEOM_ARROW,
            width=width,
            from_=from_pt.astype(np.float64),
            to=to_pt.astype(np.float64),
        )
        scn.ngeom += 1

    def _compute_gravity(self) -> np.ndarray:
        qvel_save = self._data.qvel.copy()
        qacc_save = self._data.qacc.copy()
        try:
            self._data.qvel[:] = 0.0
            self._data.qacc[:] = 0.0
            self._mj.mj_inverse(self._model, self._data)
            return self._data.qfrc_inverse.copy()
        finally:
            self._data.qvel[:] = qvel_save
            self._data.qacc[:] = qacc_save

    def _compute_ee_force(self, arm_keys, dofadr, site_id, obs, gravity):
        tau = np.array([float(obs.get(f"{k}.eff", 0.0)) for k in arm_keys], dtype=np.float64)
        if gravity is not None:
            tau = tau - gravity[dofadr]
        if not np.any(np.abs(tau) > 1e-9):
            return None
        jacp = np.zeros((3, self._model.nv), dtype=np.float64)
        jacr = np.zeros((3, self._model.nv), dtype=np.float64)
        self._mj.mj_jacSite(self._model, self._data, jacp, jacr, site_id)
        J = np.vstack([jacp[:, dofadr], jacr[:, dofadr]])
        try:
            F = np.linalg.pinv(J.T) @ tau
        except np.linalg.LinAlgError:
            return None
        return F[:3]
