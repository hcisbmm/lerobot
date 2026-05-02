# Tactile-Aware ACT Training — Design Spec

**Status:** Draft for review
**Date:** 2026-05-03
**Owner:** hcisbmm
**Scope:** Enable `lerobot-train` to consume `observation.tactile.<sensor>` features recorded by the bi-yam follower and feed them into the ACT policy as a first-class modality.

## 1. Problem

The recording pipeline already writes XELA tactile readings to LeRobotDataset under the key prefix `observation.tactile.<sensor_name>` as `(48,)` float32 tensors per sensor (see [bi_yam_follower.py:214-230,428-430](../../../src/lerobot/robots/bi_yam_follower/bi_yam_follower.py)). The training pipeline does not. Three failure points exist today:

1. **Feature classifier** ([feature_utils.py:193-194](../../../src/lerobot/utils/feature_utils.py)) blanket-tags any `observation.*` numeric key as `FeatureType.STATE`. Tactile gets misclassified.
2. **ACT model** ([modeling_act.py:340-360,460-490](../../../src/lerobot/policies/act/modeling_act.py)) only constructs encoder branches for `robot_state_feature`, `env_state_feature`, and `image_features`. Tactile is in `input_features` but not consumed by the encoder — would error at first forward pass with a missing-projection mismatch (no projection layer, never appended to `encoder_in_tokens`).
3. **Normalization map** ([configuration_act.py:88-94](../../../src/lerobot/policies/act/configuration_act.py)) defines modes for STATE/VISUAL/ACTION only. A new TACTILE feature type has no mapping.

## 2. Goals & Non-Goals

**Goals**
- ACT policy can train on datasets containing `observation.tactile.<name>` keys
- Per-sensor identity is preserved in the encoder (one transformer token per sensor)
- Backwards compatible: ACT trained without tactile keeps working unchanged
- Ablation switch: same dataset, same training script, two runs (with/without tactile) by config flag only — no code edits
- Diff stays in 5 files (no framework-wide refactor)

**Non-Goals (this spec)**
- Diffusion Policy, SmolVLA, or other policies — separate spec when needed
- Inference / `lerobot-eval` integration on real robot — covered by recording-side robot interface, no train-side blockers expected, but explicit eval validation is out of scope
- Tactile augmentation / dropout / masking — future work
- Per-sensor stat audit / alternative normalization modes — flagged in §8 as a follow-up risk

## 3. Architecture

### 3.1 Data flow (with tactile)

```
LeRobotDataset
  └─ obs.state (B, 14)
  └─ obs.images.cam_high (B, 3, H, W)
  └─ obs.tactile.right_finger_r (B, 48)   ← raw float32, recorded as-is
  └─ obs.tactile.left_finger_r  (B, 48)
  └─ action (B, chunk, 14)
        │
        ▼
dataset_to_policy_features()
  └─ classify obs.tactile.* → FeatureType.TACTILE   ← NEW
        │
        ▼
NormalizerProcessorStep (existing, generic over FeatureType)
  └─ apply MEAN_STD per tactile key (per-channel mean/std from dataset stats)
        │
        ▼
ACT.forward()
  └─ for each tactile feature in deterministic order:
       project (B, 48) → (B, dim_model)
       append to encoder_in_tokens
       append a learned positional embedding slot
  └─ stack with [latent, robot_state, env_state, image_patches]
  └─ transformer encoder → decoder → action chunk
```

### 3.2 New abstractions

**`FeatureType.TACTILE`** in [configs/types.py](../../../src/lerobot/configs/types.py:20):
- Adds enum value `TACTILE = "TACTILE"`
- Distinct from STATE so policies can selectively consume it

**`OBS_TACTILE_PREFIX = "observation.tactile."`** in [utils/constants.py](../../../src/lerobot/utils/constants.py:20):
- Single source of truth for the key prefix; mirrors `OBS_IMAGES`

**`ACTConfig.tactile_features`** property (mirrors `image_features`):
- Returns `dict[str, PolicyFeature]` filtered by `FeatureType.TACTILE`
- Defined on `PreTrainedConfig` base so any future policy can use it

**`ACTConfig.use_tactile: bool = True`** — the ablation switch:
- When `False`, tactile features are stripped from `input_features` during `__post_init__` before ACT model is constructed → no projection layers, no encoder tokens, no parameters → identical behavior to a tactile-unaware run
- Lets the user run with/without tactile on the same dataset by flipping one flag

### 3.3 Encoder integration in ACT

In [modeling_act.py](../../../src/lerobot/policies/act/modeling_act.py), tactile mirrors the `env_state` pattern:

**Constructor (`ACT.__init__`):**
```python
# After existing env_state proj setup
if self.config.tactile_features:
    self.encoder_tactile_input_proj = nn.ModuleDict({
        # Order-stable key transformation: dots → double-underscore (ModuleDict forbids dots)
        key.replace(".", "__"): nn.Linear(ft.shape[0], config.dim_model)
        for key, ft in sorted(self.config.tactile_features.items())
    })
    # Positional slots — one per tactile token, added to the existing 1d pos embed table
    n_1d_tokens += len(self.config.tactile_features)  # extends existing counter
```

**Forward (`ACT.forward`):**
```python
# After env_state token append, before image features
if self.config.tactile_features:
    for key in sorted(self.config.tactile_features):
        proj = self.encoder_tactile_input_proj[key.replace(".", "__")]
        encoder_in_tokens.append(proj(batch[key]))
```

The existing `encoder_1d_feature_pos_embed` (`nn.Embedding(n_1d_tokens, dim_model)`) auto-extends to cover the new slots — its weight rows are sliced positionally by the encoder, no per-key embedding lookup needed. Sensor identity is encoded by the position slot, which is stable because we sort keys.

### 3.4 Why sorted-key iteration

`input_features` is a `dict`. Python 3.12 dicts preserve insertion order, but insertion order depends on dataset feature registration order, which depends on user config order — fragile. `sorted()` over keys is deterministic across runs and across machines. This matters for checkpoint compatibility: if the projection ModuleDict is saved/loaded, the order of position slots must match.

### 3.5 Normalization

Add `"TACTILE": NormalizationMode.MEAN_STD` to `ACTConfig.normalization_mapping` default. The existing [`NormalizerProcessorStep._apply_transform`](../../../src/lerobot/processor/normalize_processor.py:281) is generic over `FeatureType` — it looks up `feature.type` in `norm_map` without hard-coding STATE/VISUAL. **No processor code changes required.**

Per-channel (per-taxel) mean/std is computed over the dataset by the existing stats infrastructure on the (48,) shape — same path as `observation.state`.

### 3.6 Validation

`ACTConfig.validate_features` ([configuration_act.py:162-164](../../../src/lerobot/policies/act/configuration_act.py)) currently requires "image OR env_state". Tactile is **additive** and does not satisfy this constraint by itself — keep the existing rule unchanged. (Rationale: a tactile-only ACT would be an unusual research configuration; if someone wants it, they can pass an env_state too. We don't broaden the contract speculatively.)

## 4. Files Changed

| File | Change | Why |
|---|---|---|
| `src/lerobot/configs/types.py` | Add `FeatureType.TACTILE` | New modality type |
| `src/lerobot/utils/constants.py` | Add `OBS_TACTILE_PREFIX` | Single-source key prefix |
| `src/lerobot/utils/feature_utils.py` | In `dataset_to_policy_features`, before the `OBS_STR` catch-all branch, classify keys starting with `OBS_TACTILE_PREFIX` as `FeatureType.TACTILE` | Stop misclassifying as STATE |
| `src/lerobot/configs/policies.py` | Add `tactile_features` property to `PreTrainedConfig` (filter `input_features` by `FeatureType.TACTILE`) | Reusable accessor for any policy |
| `src/lerobot/policies/act/configuration_act.py` | Add `"TACTILE": MEAN_STD` to default `normalization_mapping`; add `use_tactile: bool = True`; in `__post_init__`, when `use_tactile=False`, strip tactile keys from `self.input_features` | Wire normalization + ablation switch |
| `src/lerobot/policies/act/modeling_act.py` | Construct `encoder_tactile_input_proj` ModuleDict + extend `n_1d_tokens`; in `forward`, append projected tactile tokens after env_state, before image features (sorted by key) | Make ACT actually consume tactile |

**File count: 6** (one over the §2 goal of 5 — `configs/policies.py` adds the property; tradeoff is reusability across policies for a 4-line addition, accept).

## 5. Tests

New unit tests under `tests/`:

1. **`tests/utils/test_feature_utils.py::test_tactile_classification`**
   - Build a fake dataset features dict with `observation.tactile.foo` and `observation.tactile.bar` (shape `(48,)`, dtype `float32`)
   - Assert `dataset_to_policy_features` returns `FeatureType.TACTILE` for both keys, `FeatureType.STATE` for `observation.state`

2. **`tests/policies/test_act_tactile.py::test_act_with_tactile_forward`**
   - Build an ACTConfig with `input_features = {robot_state, image, tactile.left, tactile.right}`, `output_features = {action}`
   - Construct ACTPolicy, verify `model.encoder_tactile_input_proj` exists with 2 linears
   - Run forward with synthetic batch, verify output shape `(B, chunk_size, action_dim)` and no NaN

3. **`tests/policies/test_act_tactile.py::test_act_use_tactile_false_strips_features`**
   - Same config but `use_tactile=False`
   - Verify `policy.config.tactile_features == {}` after `__post_init__`
   - Verify `model.encoder_tactile_input_proj` does NOT exist as an attribute
   - Forward pass with a batch that has tactile keys still works (extra keys ignored by ACT)

4. **`tests/policies/test_act_tactile.py::test_act_without_tactile_unchanged`**
   - Config with no tactile in `input_features` at all
   - Verify model has no tactile-related attributes (no regression on existing ACT)
   - This test catches accidental tactile coupling for users without XELA hardware

5. **End-to-end smoke test (manual, not CI)**
   - Run `lerobot-train --policy.type=act --policy.use_tactile=true ...` against a small recorded bi-yam-tactile dataset for 100 steps
   - Verify training loss decreases, no shape mismatch, checkpoint saves and reloads

## 6. Backwards Compatibility

- **Existing ACT checkpoints (no tactile):** Load unchanged. New `use_tactile` field defaults to `True` but if `input_features` has no tactile keys, the ModuleDict is empty and no parameters are added. `n_1d_tokens` extends by 0.
- **Existing ACT checkpoints (with tactile, hypothetical future):** ModuleDict key naming uses `key.replace(".", "__")`. As long as feature keys are stable across train/eval runs, checkpoint state_dict matches.
- **Existing datasets (no tactile):** Feature classifier ignores absent prefix, no behavioral change.
- **Existing other policies (Diffusion, SmolVLA):** They never read `tactile_features`, so the new property is inert. Datasets containing tactile passed to a non-tactile-aware policy will silently include tactile keys in `input_features` — those policies will either ignore them or error at their own validation. This is acceptable for v1 because we explicitly scope to ACT; documenting in the policy class docstring is sufficient.

## 7. Failure Modes & Mitigations

| Risk | Mitigation |
|---|---|
| Raw tactile values have hardware DC offsets that vary across sensors / sessions | MEAN_STD per-key per-channel normalization absorbs offsets within a dataset; cross-dataset transfer is out of scope for v1 |
| 48-dim per sensor projected to 512-dim model = lots of params for a noisy signal | The ablation switch (`use_tactile=False`) lets you measure whether tactile actually helps — required experiment before claiming success |
| Sorted-key order changes if user renames a sensor between recording and training | Renaming sensors invalidates checkpoints regardless of order; documenting this constraint in `bi_yam_follower` config docstring is the right place, out of scope here |
| Tactile feature with non-(48,) shape (different sensor model in future) | Projection layer adapts via `nn.Linear(ft.shape[0], dim_model)` — handled |
| Dataset stats missing for tactile keys | `NormalizerProcessorStep` already raises on missing stats with a clear error — same path as STATE |

## 8. Open Questions / Follow-ups

- **Per-sensor stat audit:** Recommend running stat extraction once on the first recorded dataset and visually inspecting tactile mean/std distributions before committing to MEAN_STD. If values are extremely skewed (e.g., 99th percentile >> mean), switch to QUANTILE10. **Action: log dataset stats in first training run, eyeball before scaling experiments.**
- **Eval-time integration:** `lerobot-eval` and live robot inference need the tactile keys to be present in the obs dict at eval time. Recording side already produces them, so `lerobot-eval` against a recorded eval dataset should work. Live robot eval is a separate verification — not in this spec.
- **Other policies:** When/if expanding to Diffusion Policy, the `tactile_features` property is reusable; only the policy's encoder-equivalent (the conditioning input network) needs tactile branches added. Roughly the same diff size per policy.

## 9. Verification Plan

Pre-merge:
1. New unit tests in §5 pass
2. Existing ACT tests (`tests/policies/test_act.py` if present, else equivalent) pass unchanged
3. `pre-commit run --all-files` passes
4. End-to-end smoke run (§5 item 5) on a real recorded dataset trains for ≥100 steps without error and the loss curve is non-flat

Post-merge:
1. Train one ACT model with `use_tactile=False` and one with `use_tactile=True` on the same dataset (ablation pair)
2. Compare success rate / task metric on a held-out eval set
3. If `use_tactile=True` does not outperform, debug normalization (per §8) before assuming tactile is useless
