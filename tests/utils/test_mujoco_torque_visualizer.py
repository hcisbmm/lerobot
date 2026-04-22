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

"""Unit tests for MujocoTorqueVisualizer.

The model/data paths use real MuJoCo (skipped if the yam extra isn't installed) but the
viewer (launch_passive) and low-level render primitives (mjv_initGeom / mjv_connector)
are patched so the tests run headlessly and fast.
"""

from __future__ import annotations

from unittest.mock import MagicMock, patch

import numpy as np
import pytest

pytest.importorskip("i2rt", reason="i2rt required (install lerobot[yam])")
pytest.importorskip("mujoco", reason="mujoco required (install lerobot[yam])")

from lerobot.utils.visualization_utils import BI_YAM_JOINT_ORDER


def _make_fake_user_scn(maxgeom: int = 1024):
    scn = MagicMock()
    scn.maxgeom = maxgeom
    scn.ngeom = 0

    class _Geom:  # minimal stand-in; we only inspect attributes set on it
        pass

    scn.geoms = [_Geom() for _ in range(maxgeom)]
    return scn


@pytest.fixture
def patched_viewer_and_geom_api():
    """Patch launch_passive + mjv_initGeom + mjv_connector.

    Yields a tuple ``(init_calls, connector_calls, fake_viewer)`` where ``*_calls`` are
    lists of captured call-argument dicts (one entry per arrow), allowing tests to
    assert how many geoms were added per frame and what their colors/endpoints were.
    """
    init_calls: list[dict] = []
    connector_calls: list[dict] = []

    def _capture_init(geom, type, size, pos, mat, rgba):  # noqa: A002 — match API
        init_calls.append(
            {"geom": geom, "type": int(type), "size": size.copy(), "pos": pos.copy(),
             "mat": mat.copy(), "rgba": rgba.copy()}
        )

    def _capture_connector(geom, type, width, from_, to):  # noqa: A002
        connector_calls.append(
            {"geom": geom, "type": int(type), "width": float(width),
             "from": from_.copy(), "to": to.copy()}
        )

    fake_viewer = MagicMock()
    fake_viewer.user_scn = _make_fake_user_scn()

    with patch("mujoco.viewer.launch_passive", return_value=fake_viewer), \
         patch("mujoco.mjv_initGeom", side_effect=_capture_init), \
         patch("mujoco.mjv_connector", side_effect=_capture_connector):
        yield init_calls, connector_calls, fake_viewer


@pytest.fixture
def make_visualizer(patched_viewer_and_geom_api):
    """Factory that returns a fresh MujocoTorqueVisualizer with a very high target_fps
    so the throttle never suppresses update() during tests."""

    def _make(**kwargs):
        from lerobot.utils.visualization_utils import MujocoTorqueVisualizer
        defaults = dict(target_fps=10_000.0)
        defaults.update(kwargs)
        return MujocoTorqueVisualizer(**defaults)

    return _make


def _full_obs(torques: dict[str, float] | None = None, positions: dict[str, float] | None = None) -> dict:
    """Build a realistic obs dict with all 14 .pos and .eff keys. Unset keys default to 0."""
    obs: dict[str, float] = {}
    for name in BI_YAM_JOINT_ORDER:
        obs[f"{name}.pos"] = (positions or {}).get(name, 0.0)
        obs[f"{name}.eff"] = (torques or {}).get(name, 0.0)
    return obs


# ---------------------------------------------------------------------------
# Construction / model
# ---------------------------------------------------------------------------


def test_construction_resolves_all_joint_ids(make_visualizer):
    viz = make_visualizer()
    try:
        # All 14 canonical obs keys must resolve to valid MuJoCo joints.
        for obs_key in BI_YAM_JOINT_ORDER:
            assert obs_key in viz._joint_ids
            assert obs_key in viz._joint_qposadr
        # Arm DoF indices: 6 per side, non-overlapping.
        assert viz._left_arm_dofadr.tolist() == [0, 1, 2, 3, 4, 5]
        assert viz._right_arm_dofadr.tolist() == [8, 9, 10, 11, 12, 13]
    finally:
        viz.close()


def test_construction_requires_mujoco(monkeypatch):
    """If mujoco is not available, __init__ must raise via require_package with the yam hint."""
    import lerobot.utils.import_utils as iu

    monkeypatch.setattr(iu, "is_package_available", lambda *a, **kw: False)
    iu._require_package_cache.pop("mujoco", None)
    try:
        from lerobot.utils.visualization_utils import MujocoTorqueVisualizer

        with pytest.raises(ImportError, match=r"\[yam\]"):
            MujocoTorqueVisualizer()
    finally:
        # Always clear the poisoned cache entry so other tests see the real availability.
        iu._require_package_cache.pop("mujoco", None)


# ---------------------------------------------------------------------------
# update() behaviour
# ---------------------------------------------------------------------------


def test_update_mirrors_positions_into_qpos(make_visualizer, patched_viewer_and_geom_api):
    viz = make_visualizer()
    try:
        positions = {name: 0.1 * i for i, name in enumerate(BI_YAM_JOINT_ORDER)}
        obs = _full_obs(positions=positions)
        viz.update(obs)
        for obs_key in BI_YAM_JOINT_ORDER:
            adr = viz._joint_qposadr[obs_key]
            # Gripper obs is normalized [0,1] and scaled into the slide joint's stroke;
            # arm joints pass through unchanged.
            scale = viz._gripper_scale.get(obs_key, 1.0)
            assert viz._data.qpos[adr] == pytest.approx(positions[obs_key] * scale)
        # Gripper slave joints should mirror their masters (joint7 == joint8).
        for master, slave in viz._mirror_qposadr:
            assert viz._data.qpos[master] == viz._data.qpos[slave]
    finally:
        viz.close()


def test_gripper_obs_scaled_to_joint_stroke(make_visualizer, patched_viewer_and_geom_api):
    """Fully-open obs (1.0) must land at the slide joint's upper range, not 1 m."""
    viz = make_visualizer()
    try:
        obs = _full_obs(positions={"left_gripper": 1.0, "right_gripper": 0.0})
        viz.update(obs)
        left_id = viz._model.joint("left_joint7").id
        right_id = viz._model.joint("right_joint7").id
        left_stroke = float(viz._model.jnt_range[left_id][1] - viz._model.jnt_range[left_id][0])
        assert viz._data.qpos[viz._joint_qposadr["left_gripper"]] == pytest.approx(left_stroke)
        assert viz._data.qpos[viz._joint_qposadr["right_gripper"]] == pytest.approx(0.0)
        # Stroke came from the MJCF and must be positive but far smaller than a meter.
        assert 0.0 < left_stroke < 0.2
        assert viz._gripper_scale["left_gripper"] == pytest.approx(left_stroke)
        assert viz._gripper_scale["right_gripper"] == pytest.approx(
            float(viz._model.jnt_range[right_id][1] - viz._model.jnt_range[right_id][0])
        )
    finally:
        viz.close()


def test_update_emits_one_arrow_per_nonzero_torque(make_visualizer, patched_viewer_and_geom_api):
    init_calls, connector_calls, _viewer = patched_viewer_and_geom_api
    viz = make_visualizer(show_ee_force=False)
    try:
        # Only 3 nonzero torques
        torques = {"left_joint_0": 1.0, "left_gripper": 0.3, "right_joint_3": -0.5}
        viz.update(_full_obs(torques=torques))
        assert viz._viewer.user_scn.ngeom == 3
        assert len(init_calls) == 3
        assert len(connector_calls) == 3
    finally:
        viz.close()


def test_update_torque_arrow_color_sign(make_visualizer, patched_viewer_and_geom_api):
    init_calls, _connector, _viewer = patched_viewer_and_geom_api
    viz = make_visualizer(show_ee_force=False)
    try:
        # positive → red-ish, negative → blue-ish
        viz.update(_full_obs(torques={"left_joint_0": 2.0, "right_joint_0": -2.0}))
        colors = [call["rgba"] for call in init_calls]
        assert len(colors) == 2
        # First arrow (positive) should have dominant red; second (negative) dominant blue.
        assert colors[0][0] > colors[0][2]  # r > b
        assert colors[1][2] > colors[1][0]  # b > r
    finally:
        viz.close()


def test_update_respects_throttle(make_visualizer, patched_viewer_and_geom_api):
    init_calls, _connector, viewer = patched_viewer_and_geom_api
    viz = make_visualizer(show_ee_force=False, target_fps=1.0)  # 1 Hz → 1 s between redraws
    try:
        viz.update(_full_obs(torques={"left_joint_0": 1.0}))
        # Second call immediately after the first must be suppressed.
        viz.update(_full_obs(torques={"left_joint_0": 1.0}))
        assert viz._viewer.user_scn.ngeom == 1
        assert viewer.sync.call_count == 1
    finally:
        viz.close()


def test_update_ignores_none_obs(make_visualizer, patched_viewer_and_geom_api):
    _init, _conn, viewer = patched_viewer_and_geom_api
    viz = make_visualizer()
    try:
        viz.update(None)
        assert viewer.sync.call_count == 0
    finally:
        viz.close()


# ---------------------------------------------------------------------------
# EE-force path (Jacobian pseudoinverse with optional gravity comp)
# ---------------------------------------------------------------------------


def test_ee_force_adds_two_arrows_when_enabled(make_visualizer, patched_viewer_and_geom_api):
    init_calls, _connector, _viewer = patched_viewer_and_geom_api
    viz = make_visualizer(show_ee_force=True, compensate_gravity=False)
    try:
        # Nonzero torques on both arms so EE-force is nonzero on both sides.
        torques = {f"left_joint_{i}": 0.2 for i in range(6)}
        torques.update({f"right_joint_{i}": -0.2 for i in range(6)})
        viz.update(_full_obs(torques=torques))
        # 12 joint arrows (6 per arm) + 2 EE arrows.
        assert viz._viewer.user_scn.ngeom == 14
        # The last two arrows should use the force color (green).
        green = np.asarray([0.10, 0.75, 0.25, 0.9], dtype=np.float32)
        assert np.allclose(init_calls[-1]["rgba"], green)
        assert np.allclose(init_calls[-2]["rgba"], green)
    finally:
        viz.close()


def test_ee_force_clamped_by_max_force(make_visualizer, patched_viewer_and_geom_api):
    _init, connector_calls, _viewer = patched_viewer_and_geom_api
    # Use a very small max_ee_force so the clamp always triggers.
    viz = make_visualizer(show_ee_force=True, compensate_gravity=False, max_ee_force=1.0,
                         force_arrow_scale=1.0)
    try:
        # Huge torques → huge raw F_ee → should get clamped to max_ee_force.
        torques = {f"left_joint_{i}": 100.0 for i in range(6)}
        viz.update(_full_obs(torques=torques))
        # Find the EE-force connector call (last arrow on the left side). The
        # connector records from/to; distance between them is force_arrow_scale * clamped_mag.
        # Because arrow_scale is 1.0 and max is 1.0, arrow length must be ≤ 1.0.
        lengths = [float(np.linalg.norm(call["to"] - call["from"])) for call in connector_calls]
        ee_arrow_len = lengths[-1]  # last arrow added = EE force on the right (or left, whichever came last)
        assert ee_arrow_len <= 1.0 + 1e-6
    finally:
        viz.close()


def test_gravity_compensation_toggle_changes_force(make_visualizer, patched_viewer_and_geom_api):
    """With gravity comp ON vs OFF, the rendered EE-force arrows should differ in magnitude
    on at least one side (unless we happen to be at a zero-gravity pose, which we aren't)."""
    from lerobot.utils.visualization_utils import MujocoTorqueVisualizer

    # Use an unusual arm pose so g(q) is clearly nonzero.
    pos = {f"left_joint_{i}": 0.4 for i in range(6)}
    pos.update({f"right_joint_{i}": 0.4 for i in range(6)})
    obs = _full_obs(positions=pos, torques={})  # zero measured torque
    # With compensate_gravity=False, (τ_meas − 0) = 0 → F_ee = 0 → no EE arrow rendered.
    # With compensate_gravity=True, τ_ext = 0 − g(q) = −g(q) → F_ee should be nonzero.
    viz_off = make_visualizer(show_ee_force=True, compensate_gravity=False)
    try:
        viz_off.update(obs)
        ngeom_off = viz_off._viewer.user_scn.ngeom
    finally:
        viz_off.close()

    # Fresh visualizer for the compensated case (separate fixture reset).
    with patch("mujoco.viewer.launch_passive") as mock_launch, \
         patch("mujoco.mjv_initGeom"), patch("mujoco.mjv_connector"):
        fake_viewer = MagicMock()
        fake_viewer.user_scn = _make_fake_user_scn()
        mock_launch.return_value = fake_viewer
        viz_on = MujocoTorqueVisualizer(show_ee_force=True, compensate_gravity=True,
                                        target_fps=10_000.0)
        try:
            viz_on.update(obs)
            ngeom_on = viz_on._viewer.user_scn.ngeom
        finally:
            viz_on.close()

    # With zero measured τ and no compensation, there should be zero EE arrows.
    # With compensation on, there should be at least one (one per arm with nonzero g).
    assert ngeom_off == 0
    assert ngeom_on > 0
