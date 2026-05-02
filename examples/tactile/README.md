# Tactile Inspection Examples

Utilities for examining tactile data captured by `lerobot-record` with an XELA
sensor. Related docs:

- [`src/lerobot/tactile/`](../../src/lerobot/tactile/) — subsystem overview,
  data contract, and adding new sensor backends.
- [`src/lerobot/tactile/xela/README.md`](../../src/lerobot/tactile/xela/README.md)
  — XELA wire protocol, vendor first-time setup (apt / `/etc/xela` / unpack
  `appimage.zip` / interactive `xela_conf`), per-boot bring-up, and the full
  failure-modes reference.
- [`src/lerobot/robots/bi_yam_follower/README.md`](../../src/lerobot/robots/bi_yam_follower/README.md#tactile-sensor-xela-optional)
  — recording-side usage on the bimanual Yam follower.

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

| `--mode`     | What it shows                                                                                                                                                           |
| ------------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `summary`    | Shape / dtype / per-axis (X, Y, Z) min, max, mean, std over the episode. **No GUI required** — useful in CI or over SSH.                                                |
| `timeseries` | Three stacked line plots (X, Y, Z) with one line per taxel over the episode. Spikes correspond to contact events.                                                       |
| `heatmap`    | Animated 4×4 grid of `‖Δ from baseline‖` magnitudes. Bright cells light up when the gripper makes contact.                                                              |
| `frame`      | Single mid-episode frame: gripper camera image side-by-side with the tactile heatmap. Good for sanity-checking that contact in the video lines up with sensor activity. |
| `all`        | Runs `summary`, then opens each plot in turn.                                                                                                                           |

### Flags

- `--repo-id <hf-id>` (required) — `${HF_USER}/<dataset_name>`.
- `--episode <int>` — episode index (default `0`).
- `--key <name>` — specific `observation.tactile.<name>` key when more than
  one sensor is recorded. Default = first alphabetically.
- `--root <path>` — read directly from a local dataset root instead of
  fetching from the Hub. Useful when the dataset isn't published yet, when
  you want to inspect a local snapshot in place, or when you have a
  writable working copy outside the Hub cache.

### Dependencies

The headless `summary` mode needs only the core install (`uv sync --locked`).

The plot modes (`timeseries`, `heatmap`, `frame`, `all`) need `matplotlib`,
which lives behind the `matplotlib-dep` extra (also pulled in by `dev`):

```bash
uv sync --locked --extra matplotlib-dep    # minimal: just matplotlib
# or, if you already work with the dev tooling:
uv sync --locked --extra dev
```
