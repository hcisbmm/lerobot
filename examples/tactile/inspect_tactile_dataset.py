#!/usr/bin/env python
# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
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

"""Inspect, verify, and visualise XELA tactile data in a LeRobot dataset.

Designed for datasets recorded with `lerobot-record --robot.tactile_sensors=...`
where one or more keys of the form `observation.tactile.<name>` are present.

Usage::

    uv run python examples/tactile/inspect_tactile_dataset.py \
      --repo-id ${HF_USER}/bimanual-yam-tactile-demo \
      --episode 0 \
      --mode all

Modes
-----
- ``summary``    — print schema + per-channel min/max/mean over the episode (no plots)
- ``timeseries`` — line plot, all 16 taxels split into X/Y/Z subplots
- ``heatmap``    — animated 4×4 force-magnitude grid (delta from baseline)
- ``frame``      — single-frame side-by-side: gripper camera + tactile heatmap
- ``all``        — everything above, in sequence

If multiple tactile keys are present (e.g. ``right_finger_r``, ``left_finger_r``),
pass ``--key`` to pick one. Otherwise the first one alphabetically is used.
"""

from __future__ import annotations

import argparse
import sys
from collections.abc import Iterator
from pathlib import Path

import numpy as np
import torch

from lerobot.datasets import LeRobotDataset

DEFAULT_TAXELS_PER_AXIS = 4  # XR1944 layout
AXES_PER_TAXEL = 3


def _tactile_keys(ds: LeRobotDataset) -> list[str]:
    return sorted(k for k in ds.meta.features if k.startswith("observation.tactile."))


def _as_numpy(v) -> np.ndarray:
    if isinstance(v, torch.Tensor):
        return v.detach().cpu().numpy()
    return np.asarray(v)


def _iter_tactile(ds: LeRobotDataset, key: str) -> Iterator[np.ndarray]:
    """Yield each frame's tactile vector as a 1-D numpy array."""
    for i in range(len(ds)):
        yield _as_numpy(ds[i][key])


def _stack_tactile(ds: LeRobotDataset, key: str) -> np.ndarray:
    """Return stacked tactile array of shape (T, N) where N = num_taxels * 3."""
    return np.stack(list(_iter_tactile(ds, key)))


def _camera_keys(ds: LeRobotDataset) -> list[str]:
    return sorted(
        k for k in ds.meta.features if k.startswith("observation.images.")
    )


def cmd_summary(ds: LeRobotDataset, key: str) -> None:
    print("=" * 60)
    print(f"dataset:    {ds.meta.repo_id}")
    print(f"fps:        {ds.fps}")
    print(f"frames:     {len(ds)}  (episode-filtered view)")
    print(f"key:        {key}")
    feat = ds.meta.features[key]
    print(f"shape:      {feat.get('shape')}")
    print(f"dtype:      {feat.get('dtype')}")
    print()

    arr = _stack_tactile(ds, key)  # (T, N)
    n = arr.shape[1]
    print(f"observations: T={arr.shape[0]} channels={n}")

    if n % AXES_PER_TAXEL == 0:
        taxels = n // AXES_PER_TAXEL
        per_axis = arr.reshape(arr.shape[0], taxels, AXES_PER_TAXEL)
        for axis_idx, axis_name in enumerate(("X", "Y", "Z")):
            slab = per_axis[:, :, axis_idx]
            print(f"  axis {axis_name}: "
                  f"min={slab.min():8.1f} max={slab.max():8.1f} "
                  f"mean={slab.mean():8.1f} std={slab.std():7.2f}")
    else:
        print(f"  flat: min={arr.min():.1f} max={arr.max():.1f} "
              f"mean={arr.mean():.1f} std={arr.std():.2f}")
    print("=" * 60)


def cmd_timeseries(ds: LeRobotDataset, key: str) -> None:
    import matplotlib.pyplot as plt

    arr = _stack_tactile(ds, key)
    if arr.shape[1] % AXES_PER_TAXEL != 0:
        raise ValueError(
            f"channel count {arr.shape[1]} is not divisible by {AXES_PER_TAXEL}; "
            "cannot split into X/Y/Z."
        )
    T = arr.shape[0]
    taxels = arr.shape[1] // AXES_PER_TAXEL
    per_axis = arr.reshape(T, taxels, AXES_PER_TAXEL)
    ts = np.arange(T) / ds.fps

    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    for axis_idx, name in enumerate(("X", "Y", "Z")):
        for taxel in range(taxels):
            axes[axis_idx].plot(ts, per_axis[:, taxel, axis_idx],
                                alpha=0.5, lw=0.8)
        axes[axis_idx].set_ylabel(f"{name}")
        axes[axis_idx].grid(True, alpha=0.3)
    axes[-1].set_xlabel("seconds")
    fig.suptitle(f"{key} — {T} frames @ {ds.fps} Hz, {taxels} taxels")
    plt.tight_layout()
    plt.show()


def cmd_heatmap(ds: LeRobotDataset, key: str) -> None:
    import matplotlib.animation as animation
    import matplotlib.pyplot as plt

    arr = _stack_tactile(ds, key)
    if arr.shape[1] != DEFAULT_TAXELS_PER_AXIS ** 2 * AXES_PER_TAXEL:
        # Generic reshape for non-XR1944 layouts.
        taxels = arr.shape[1] // AXES_PER_TAXEL
        side = int(round(taxels ** 0.5))
        if side * side != taxels:
            raise ValueError(
                f"channel count {arr.shape[1]} doesn't fit a square taxel layout; "
                "skip --mode heatmap or pass a key with a square grid."
            )
    else:
        side = DEFAULT_TAXELS_PER_AXIS

    T = arr.shape[0]
    grid_3d = arr.reshape(T, side * side, AXES_PER_TAXEL)
    baseline = grid_3d[0]  # use first frame as zero
    delta_mag = np.linalg.norm(grid_3d - baseline, axis=2).reshape(T, side, side)

    vmax = max(50.0, float(delta_mag.max()))
    fig, ax = plt.subplots(figsize=(5, 5))
    im = ax.imshow(delta_mag[0], vmin=0, vmax=vmax,
                   cmap="magma", interpolation="nearest")
    ax.set_title(f"{key} — |Δ from baseline|")
    fig.colorbar(im, ax=ax, label="‖Δ raw‖")
    txt = ax.set_xlabel(f"frame 0 (0.00 s)")

    def update(t):
        im.set_data(delta_mag[t])
        txt.set_text(f"frame {t} ({t / ds.fps:.2f} s)")
        return [im, txt]

    interval_ms = max(10, int(1000 / ds.fps))
    ani = animation.FuncAnimation(  # noqa: F841 — ref keeps animation alive
        fig, update, frames=range(T), interval=interval_ms, blit=False
    )
    plt.show()


def cmd_frame(ds: LeRobotDataset, key: str) -> None:
    import matplotlib.pyplot as plt

    cam_keys = _camera_keys(ds)
    if not cam_keys:
        print("no camera keys found; falling back to tactile-only view", file=sys.stderr)
        return cmd_heatmap(ds, key)

    cam_key = cam_keys[0]
    mid = len(ds) // 2
    sample = ds[mid]
    img = _as_numpy(sample[cam_key])
    if img.ndim == 3 and img.shape[0] in (1, 3):  # CHW → HWC
        img = np.transpose(img, (1, 2, 0))
    if img.dtype != np.uint8:
        img = (img * 255).clip(0, 255).astype(np.uint8) if img.max() <= 1.0 else img.astype(np.uint8)

    tactile = _as_numpy(sample[key])
    taxels = tactile.size // AXES_PER_TAXEL
    side = int(round(taxels ** 0.5))
    grid = tactile.reshape(taxels, AXES_PER_TAXEL)
    mag = np.linalg.norm(grid - grid.mean(axis=0), axis=1).reshape(side, side)

    fig, (a1, a2) = plt.subplots(1, 2, figsize=(12, 5))
    a1.imshow(img); a1.set_title(f"{cam_key}, frame {mid}"); a1.axis("off")
    h = a2.imshow(mag, cmap="magma", interpolation="nearest")
    a2.set_title(f"{key} |Δ from frame mean|")
    fig.colorbar(h, ax=a2)
    plt.tight_layout()
    plt.show()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--repo-id", required=True,
                    help="HuggingFace repo id, e.g. 'HGLLL/bimanual-yam-tactile-demo'.")
    ap.add_argument("--episode", type=int, default=0,
                    help="Episode index to inspect (default: 0).")
    ap.add_argument("--key", default=None,
                    help="Specific tactile key (default: first one alphabetically).")
    ap.add_argument("--mode", default="summary",
                    choices=["summary", "timeseries", "heatmap", "frame", "all"],
                    help="Inspection mode.")
    ap.add_argument("--root", type=Path, default=None,
                    help="Optional local dataset root override.")
    args = ap.parse_args()

    ds = LeRobotDataset(
        args.repo_id,
        episodes=[args.episode],
        root=args.root,
    )

    keys = _tactile_keys(ds)
    if not keys:
        print(f"error: no `observation.tactile.*` keys in {args.repo_id}", file=sys.stderr)
        print(f"available features: {sorted(ds.meta.features)}", file=sys.stderr)
        return 1

    key = args.key or keys[0]
    if key not in keys:
        print(f"error: key {key!r} not in dataset. Available tactile keys: {keys}",
              file=sys.stderr)
        return 1

    print(f"loaded {args.repo_id} episode={args.episode} key={key}\n")

    if args.mode in ("summary", "all"):
        cmd_summary(ds, key)
    if args.mode in ("timeseries", "all"):
        cmd_timeseries(ds, key)
    if args.mode in ("heatmap", "all"):
        cmd_heatmap(ds, key)
    if args.mode in ("frame", "all"):
        cmd_frame(ds, key)

    return 0


if __name__ == "__main__":
    sys.exit(main())
