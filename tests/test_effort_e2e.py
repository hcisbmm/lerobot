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

"""
End-to-end tests for effort/torque (.eff) recording, training, and visualization.

Uses MockRobotWithEffort + MockTeleop to validate the full pipeline without hardware.
"""

import importlib
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from tests.mocks.mock_robot_with_effort import MockRobotWithEffort, MockRobotWithEffortConfig
from tests.mocks.mock_teleop import MockTeleop, MockTeleopConfig

DUMMY_REPO_ID = "dummy/effort_test"
N_MOTORS = 3


# ---------------------------------------------------------------------------
# Recording integration tests
# ---------------------------------------------------------------------------


def _make_robot_and_teleop():
    """Create and connect a MockRobotWithEffort and MockTeleop pair."""
    robot_cfg = MockRobotWithEffortConfig(n_motors=N_MOTORS)
    robot = MockRobotWithEffort(robot_cfg)
    robot.connect()

    teleop_cfg = MockTeleopConfig(n_motors=N_MOTORS)
    teleop = MockTeleop(teleop_cfg)
    teleop.connect()

    return robot, teleop


def _record_mini_dataset(tmp_path, robot, teleop, record_effort):
    """Record a mini dataset with the given record_effort setting."""
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.datasets.pipeline_features import aggregate_pipeline_dataset_features, create_initial_features
    from lerobot.datasets.utils import build_dataset_frame, combine_feature_dicts
    from lerobot.processor import make_default_processors
    from lerobot.utils.constants import ACTION, OBS_STR

    teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

    # Filter .eff keys from observation features when record_effort is disabled
    obs_features = robot.observation_features
    if not record_effort:
        obs_features = {k: v for k, v in obs_features.items() if not k.endswith(".eff")}

    dataset_features = combine_feature_dicts(
        aggregate_pipeline_dataset_features(
            pipeline=teleop_action_processor,
            initial_features=create_initial_features(action=robot.action_features),
            use_videos=False,
        ),
        aggregate_pipeline_dataset_features(
            pipeline=robot_observation_processor,
            initial_features=create_initial_features(observation=obs_features),
            use_videos=False,
        ),
    )

    dataset = LeRobotDataset.create(
        DUMMY_REPO_ID,
        fps=30,
        root=tmp_path / "_dataset",
        robot_type=robot.name,
        features=dataset_features,
        use_videos=False,
    )

    # Record 2 episodes with 5 frames each
    for _ep in range(2):
        for _ in range(5):
            obs = robot.get_observation()
            if not record_effort:
                obs = {k: v for k, v in obs.items() if not k.endswith(".eff")}

            obs_processed = robot_observation_processor(obs)
            observation_frame = build_dataset_frame(dataset.features, obs_processed, prefix=OBS_STR)

            action = teleop.get_action()
            action_processed = teleop_action_processor((action, obs))
            action_frame = build_dataset_frame(dataset.features, action_processed, prefix=ACTION)

            frame = {**observation_frame, **action_frame, "task": "test_task"}
            dataset.add_frame(frame)
        dataset.save_episode()

    dataset.finalize()
    return dataset


def test_record_with_effort(tmp_path):
    """Record a mini dataset with record_effort=True, verify .eff keys in features and data."""
    robot, teleop = _make_robot_and_teleop()
    try:
        dataset = _record_mini_dataset(tmp_path, robot, teleop, record_effort=True)

        # Check observation.state feature includes .eff names
        state_ft = dataset.features["observation.state"]
        names = state_ft["names"]
        eff_names = [n for n in names if n.endswith(".eff")]
        pos_names = [n for n in names if n.endswith(".pos")]

        assert len(eff_names) == N_MOTORS, f"Expected {N_MOTORS} .eff names, got {len(eff_names)}"
        assert len(pos_names) == N_MOTORS, f"Expected {N_MOTORS} .pos names, got {len(pos_names)}"

        # observation.state shape should be n_pos + n_eff
        assert state_ft["shape"] == (N_MOTORS * 2,), f"Expected shape ({N_MOTORS * 2},), got {state_ft['shape']}"

        # action shape should be n_pos only
        action_ft = dataset.features["action"]
        assert action_ft["shape"] == (N_MOTORS,), f"Expected action shape ({N_MOTORS},), got {action_ft['shape']}"

        # Read back data and verify .eff values exist (non-zero for random data)
        item = dataset[0]
        obs_state = item["observation.state"]
        assert obs_state.shape[-1] == N_MOTORS * 2
    finally:
        robot.disconnect()
        teleop.disconnect()


def test_record_without_effort_backward_compat(tmp_path):
    """Record with record_effort=False, verify no .eff in features (backward compatible)."""
    robot, teleop = _make_robot_and_teleop()
    try:
        dataset = _record_mini_dataset(tmp_path, robot, teleop, record_effort=False)

        # Check observation.state feature has NO .eff names
        state_ft = dataset.features["observation.state"]
        names = state_ft["names"]
        eff_names = [n for n in names if n.endswith(".eff")]
        pos_names = [n for n in names if n.endswith(".pos")]

        assert len(eff_names) == 0, f"Expected 0 .eff names, got {len(eff_names)}"
        assert len(pos_names) == N_MOTORS

        # observation.state shape should be n_pos only
        assert state_ft["shape"] == (N_MOTORS,)

        # action shape should be n_pos only
        action_ft = dataset.features["action"]
        assert action_ft["shape"] == (N_MOTORS,)
    finally:
        robot.disconnect()
        teleop.disconnect()


# ---------------------------------------------------------------------------
# Training E2E test
# ---------------------------------------------------------------------------


def test_policy_discovers_effort_state_shape(tmp_path):
    """Create a dataset with .eff in observation.state, verify TDMPC auto-discovers correct shapes."""
    pytest.importorskip("torch")

    from lerobot.configs.types import FeatureType, PolicyFeature
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.policies.factory import make_policy, make_policy_config

    n_state = N_MOTORS * 2  # pos + eff
    n_action = N_MOTORS

    # Create a dummy dataset with .eff in observation.state
    features = {
        "action": {
            "dtype": "float32",
            "shape": (n_action,),
            "names": [f"motor_{i+1}.pos" for i in range(n_action)],
        },
        "observation.state": {
            "dtype": "float32",
            "shape": (n_state,),
            "names": [f"motor_{i+1}.pos" for i in range(n_action)]
            + [f"motor_{i+1}.eff" for i in range(N_MOTORS)],
        },
    }
    dataset = LeRobotDataset.create(
        DUMMY_REPO_ID,
        fps=30,
        root=tmp_path / "_dataset",
        features=features,
        use_videos=False,
    )

    for ep_idx in range(2):
        for _ in range(5):
            frame = {
                "action": np.random.randn(n_action).astype(np.float32),
                "observation.state": np.random.randn(n_state).astype(np.float32),
                "task": f"task_{ep_idx}",
            }
            dataset.add_frame(frame)
        dataset.save_episode()
    dataset.finalize()

    # Create TDMPC policy config — the key test is that make_policy auto-discovers
    # the enlarged observation.state shape (pos + eff)
    policy_cfg = make_policy_config(
        "tdmpc",
        device="cpu",
        push_to_hub=False,
    )

    # Instantiate policy from dataset metadata — this should auto-discover shapes
    policy = make_policy(policy_cfg, ds_meta=dataset.meta)

    # Verify the policy discovered the correct input/output shapes
    state_feature = policy.config.input_features["observation.state"]
    assert state_feature.shape == (n_state,), (
        f"Policy should auto-discover state shape ({n_state},), got {state_feature.shape}"
    )
    assert state_feature.type == FeatureType.STATE

    action_feature = policy.config.output_features["action"]
    assert action_feature.shape == (n_action,), (
        f"Policy should auto-discover action shape ({n_action},), got {action_feature.shape}"
    )
    assert action_feature.type == FeatureType.ACTION

    # Verify dataset readback: .eff values are present in observation.state
    item = dataset[0]
    obs_state = item["observation.state"]
    assert obs_state.shape[-1] == n_state, f"Expected state dim {n_state}, got {obs_state.shape[-1]}"

    # Verify the action dimension is correct (pos only, no eff)
    action = item["action"]
    assert action.shape[-1] == n_action, f"Expected action dim {n_action}, got {action.shape[-1]}"


# ---------------------------------------------------------------------------
# Rerun visualization tests
# ---------------------------------------------------------------------------


@pytest.fixture
def mock_rerun(monkeypatch):
    """Provide a mock `rerun` module so tests don't depend on the real library."""
    calls = []

    class DummyScalar:
        def __init__(self, value):
            self.value = float(value)

    class DummyImage:
        def __init__(self, arr):
            self.arr = arr

    def dummy_log(key, obj=None, **kwargs):
        if obj is None and "entity" in kwargs:
            obj = kwargs.pop("entity")
        calls.append((key, obj, kwargs))

    dummy_rr = SimpleNamespace(
        Scalars=DummyScalar,
        Image=DummyImage,
        log=dummy_log,
        init=lambda *a, **k: None,
        spawn=lambda *a, **k: None,
    )

    monkeypatch.setitem(sys.modules, "rerun", dummy_rr)

    import lerobot.utils.visualization_utils as vu

    importlib.reload(vu)

    yield vu, calls


def test_rerun_logs_effort_keys(mock_rerun):
    """Verify that .eff keys from observation are logged as Scalars in Rerun."""
    vu, calls = mock_rerun

    obs_with_eff = {
        "left_joint_0.pos": 1.0,
        "left_joint_0.eff": 0.5,
        "right_joint_0.pos": 2.0,
        "right_joint_0.eff": -0.3,
        "right_gripper.eff": 1.2,
    }

    vu.log_rerun_data(observation=obs_with_eff)

    logged_keys = {k for k, _obj, _kw in calls}

    # All .eff keys should appear as observation scalars
    assert "observation.left_joint_0.eff" in logged_keys
    assert "observation.right_joint_0.eff" in logged_keys
    assert "observation.right_gripper.eff" in logged_keys

    # .pos keys should also be logged
    assert "observation.left_joint_0.pos" in logged_keys
    assert "observation.right_joint_0.pos" in logged_keys

    # Verify they are Scalars
    for k, obj, _kw in calls:
        if ".eff" in k:
            assert type(obj).__name__ == "DummyScalar"


def test_rerun_rrd_contains_effort_timeseries():
    """Test that .eff scalars can be written and read back from a Rerun .rrd file."""
    rr = pytest.importorskip("rerun")

    try:
        rr_df = pytest.importorskip("rerun.dataframe")
    except Exception:
        pytest.skip("rerun.dataframe not available")

    import gc
    import tempfile
    from pathlib import Path

    with tempfile.TemporaryDirectory() as tmpdir:
        rrd_path = str(Path(tmpdir) / "test.rrd")

        rec = rr.RecordingStream("effort_test")
        rec.save(rrd_path)

        eff_keys = ["left_joint_0/eff", "right_joint_0/eff", "right_gripper/eff"]
        n_steps = 10

        for step in range(n_steps):
            rec.set_time_sequence("step", step)
            for key in eff_keys:
                rec.log(f"observation/{key}", rr.Scalars(float(step * 0.1)))

        # Ensure all data is flushed to disk
        rec.flush()
        del rec
        gc.collect()

        # Read back
        recording = rr_df.load_recording(rrd_path)
        view = recording.view(index="step", contents="/**")
        table = view.select().read_all()
        column_names = [c.name for c in table.schema]

        # Check that .eff columns exist (Rerun uses / separators in entity paths)
        for key in eff_keys:
            matching = [c for c in column_names if key in c]
            assert len(matching) > 0, f"Expected column for {key}, found columns: {column_names}"
