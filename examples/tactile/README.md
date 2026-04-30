# Tactile Inspection Examples

Utilities for examining tactile data captured by `lerobot-record` with an XELA
sensor. See [`src/lerobot/tactile/`](../../src/lerobot/tactile/) for the
subsystem and [`src/lerobot/robots/bi_yam_follower/README.md`](../../src/lerobot/robots/bi_yam_follower/README.md#tactile-sensor-xela-optional)
for the recording instructions.

## `inspect_tactile_dataset.py`

Examine, verify, and visualise the `observation.tactile.<name>` columns
recorded into a LeRobot dataset.

### Quick start

```bash
# Schema + per-axis stats, no plots — fastest sanity check
uv run python examples/tactile/inspect_tactile_dataset.py \
  --repo-id ${HF_USER}/bimanual-yam-tactile-demo --episode 0

# All views at once: summary, time-series, animated heatmap, frame side-by-side
uv run python examples/tactile/inspect_tactile_dataset.py \
  --repo-id ${HF_USER}/bimanual-yam-tactile-demo --episode 0 --mode all
```

### Modes

| `--mode`    | What it shows |
| ---         | --- |
| `summary`   | Shape / dtype / per-axis (X, Y, Z) min, max, mean, std over the episode. **No GUI required** — useful in CI or over SSH. |
| `timeseries`| Three stacked line plots (X, Y, Z) with one line per taxel over the episode. Spikes correspond to contact events. |
| `heatmap`   | Animated 4×4 grid of `‖Δ from baseline‖` magnitudes. Bright cells light up when the gripper makes contact. |
| `frame`     | Single mid-episode frame: gripper camera image side-by-side with the tactile heatmap. Good for sanity-checking that contact in the video lines up with sensor activity. |
| `all`       | Runs `summary`, then opens each plot in turn. |

### Flags

- `--repo-id <hf-id>` (required) — `${HF_USER}/<dataset_name>`.
- `--episode <int>` — episode index (default `0`).
- `--key <name>` — specific `observation.tactile.<name>` key when more than
  one sensor is recorded. Default = first alphabetically.
- `--root <path>` — local dataset root override (skips Hub fetch when you
  already have it cached or stored locally).

### Dependencies

`matplotlib` is needed for the plot modes; it ships with the project's `dev`
extras. If you only want the headless `summary` mode, no extra installs
beyond the core deps are required.
