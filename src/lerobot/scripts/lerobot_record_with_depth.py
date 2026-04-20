#!/usr/bin/env python

# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
"""
Record a dataset with depth map video in addition to RGB.

Thin wrapper around `lerobot.scripts.lerobot_record.record` that:
  1. Attaches depth-stream capture to the robot in-place: any camera listed
     in --depth_cams emits a `{cam}_depth` observation key alongside its
     regular color frame. The camera must be configured with use_depth=True.
  2. Patches `LeRobotDataset.create` to:
       * inject depth_map_encoding_fn / depth_map_decoding_fn, and
       * stamp `video.is_depth_map=True` (plus codec/pix_fmt/crf) on every
         depth feature's info dict so the writer routes them through the
         lossless depth pipeline.

Depth packing follows the reference recipe from
`docs/source/streaming_video_encoding.mdx`: log-quantize depth (meters) to
12 bits and store in the Y plane of yuv420p12le. RealSense returns uint16
millimeters, so we divide by 1000 before quantizing.

Usage mirrors `lerobot-record`, with one extra flag:
  --depth_cams='[cam1, cam2]'

Example (single-arm smoke test):
    python scripts/record_with_depth.py \\
        --robot.type=bi_yam_follower \\
        --teleop.type=bi_yam_leader \\
        --dataset.repo_id=${HF_USER}/bi-yam-depth-smoke \\
        --dataset.num_episodes=2 \\
        --dataset.single_task='Grab the cube' \\
        --dataset.streaming_encoding=true \\
        --dataset.encoder_threads=2 \\
        --depth_cams='[front]'
"""

import logging
import math
from contextlib import contextmanager
from dataclasses import dataclass, field
from pprint import pformat

import av
import numpy as np
import torch
from lerobot.configs import parser
from lerobot.datasets import LeRobotDataset
from lerobot.scripts.lerobot_record import RecordConfig, record
from lerobot.utils.utils import init_logging

# `record` is decorated with @parser.wrap(); its wrapper checks
# `type(args[0]) is RecordConfig` (strict, not isinstance), so passing a
# DepthRecordConfig subclass instance triggers a re-parse of sys.argv as
# the parent class — which doesn't know `--depth_cams` and argparse rejects
# the whole invocation. Unwrap to call the underlying function directly.
_record_impl = getattr(record, "__wrapped__", record)

logger = logging.getLogger(__name__)


# ────────────────────────────────────────────────────────────────
# Depth codec choice
# ────────────────────────────────────────────────────────────────
# libsvtav1 in common ffmpeg builds accepts yuv420p10le (10-bit) but often
# not yuv420p12le. We stay at 10 bits to match the widely-shipped codec.
# 10-bit log-quantization over 0.1–3.0 m gives sub-cm precision near 3 m
# and sub-mm near 0.1 m — sufficient for most bimanual manipulation tasks.
DEPTH_CODEC = "libsvtav1"
DEPTH_PIX_FMT = "yuv420p10le"
DEPTH_CRF = 0  # lossless given fixed pix_fmt

# Default scene depth range (meters). Overridable via --min_depth_m /
# --max_depth_m. Narrow this to the actual working volume — log-scale
# quantization puts more bits near the close range, so 0.1–1.0 m gives
# ~3× tighter per-level precision than 0.1–3.0 m.
DEFAULT_MIN_DEPTH_M = 0.10
DEFAULT_MAX_DEPTH_M = 3.0
DEPTH_SHIFT = 1.0  # log shift; avoids log(0) and keeps dynamic range sensible
Q_BITS = 10
Q_MAX = (1 << Q_BITS) - 1

# Module-level state that the encode/decode functions close over. Set by
# `make_depth_codec(...)` or the wrapper's `_inject_depth_pipeline(...)`.
# For simplicity (single source of truth) we treat these as module globals
# rather than threading them through every call.
MIN_DEPTH_M = DEFAULT_MIN_DEPTH_M
MAX_DEPTH_M = DEFAULT_MAX_DEPTH_M


# ────────────────────────────────────────────────────────────────
# Depth encode / decode (10-bit log-quantize ↔ yuv420p10le)
# ────────────────────────────────────────────────────────────────
def _log_bounds() -> tuple[float, float]:
    return math.log(MIN_DEPTH_M + DEPTH_SHIFT), math.log(MAX_DEPTH_M + DEPTH_SHIFT)


def _quantize_depth_m(depth_m: np.ndarray) -> np.ndarray:
    log_min, log_max = _log_bounds()
    log_depth = np.log(np.clip(depth_m, MIN_DEPTH_M, MAX_DEPTH_M) + DEPTH_SHIFT)
    log_norm = (log_depth - log_min) / (log_max - log_min)
    return np.clip((log_norm * Q_MAX).round(), 0, Q_MAX).astype(np.uint16)


def _dequantize_depth_m(q: torch.Tensor) -> torch.Tensor:
    log_min, log_max = _log_bounds()
    log_norm = q.float() / Q_MAX
    log_depth = log_norm * (log_max - log_min) + log_min
    return torch.clamp(torch.exp(log_depth) - DEPTH_SHIFT, MIN_DEPTH_M, MAX_DEPTH_M)


def set_depth_range(min_m: float, max_m: float) -> None:
    """Set the active depth range used by `encode_depth` / `decode_depth`.

    Must be called before `LeRobotDataset.create(...)` at record time AND
    before `LeRobotDataset(...)` at read time — the range is NOT stored
    inside the mp4, only inside the dataset's feature info dict (see
    `video.depth_min_m` / `video.depth_max_m` stamped by the wrapper).
    """
    global MIN_DEPTH_M, MAX_DEPTH_M
    if not (0.0 < min_m < max_m):
        raise ValueError(
            f"Invalid depth range: min={min_m}, max={max_m}. Must satisfy 0 < min < max."
        )
    MIN_DEPTH_M = float(min_m)
    MAX_DEPTH_M = float(max_m)
    logger.info("Depth range set to [%.3f, %.3f] m", MIN_DEPTH_M, MAX_DEPTH_M)


def encode_depth(depth: np.ndarray) -> av.VideoFrame:
    """RealSense-style uint16 mm (or float meters) depth → yuv420p10le frame.

    Accepted shapes: (H, W), (H, W, 1). Y plane carries the 10-bit log-
    quantized depth; chroma is set to neutral (2^9). Range is read from
    module globals MIN_DEPTH_M / MAX_DEPTH_M — use `set_depth_range(...)`
    to override before recording or loading.
    """
    if depth.ndim == 3:
        depth = depth[..., 0]
    if depth.dtype == np.uint16:
        # Assume RealSense convention: millimeters → meters.
        depth_m = depth.astype(np.float32) / 1000.0
    else:
        depth_m = depth.astype(np.float32)

    q = _quantize_depth_m(depth_m)
    h, w = q.shape
    frame = av.VideoFrame(width=w, height=h, format=DEPTH_PIX_FMT)
    frame.planes[0].update(q.tobytes())
    neutral = np.full((h // 2, w // 2), 1 << (Q_BITS - 1), dtype=np.uint16)
    frame.planes[1].update(neutral.tobytes())
    frame.planes[2].update(neutral.tobytes())
    return frame


def _read_y_plane_u16(frame: av.VideoFrame) -> np.ndarray:
    """Lossless read of the Y plane as uint16 (frame must be yuv420pXle).

    pyav's `reformat(format='gray*le').to_ndarray()` applies limited-range YUV
    -> full-range conversion and clips values near zero; we bypass that by
    reading raw plane bytes directly.
    """
    plane = frame.planes[0]
    raw = np.frombuffer(bytes(plane), dtype=np.uint16)
    stride_u16 = plane.line_size // 2
    return raw.reshape(frame.height, stride_u16)[:, : frame.width].copy()


def decode_depth(frames: list[av.VideoFrame]) -> torch.Tensor:
    """yuv420p12le frames → torch.Tensor (N, H, W) float32 meters."""
    q = torch.from_numpy(np.stack([_read_y_plane_u16(f) for f in frames]))
    return _dequantize_depth_m(q)


# ────────────────────────────────────────────────────────────────
# Robot adapter — in-place mutation (no wrapper class; avoids isinstance
# pitfalls inside record_loop).
# ────────────────────────────────────────────────────────────────
def _attach_depth_capability(robot, depth_cams: list[str]):
    """Add `{cam}_depth` to observation_features + get_observation in place.

    Assumes the robot's `observation_features` is a @cached_property (true for
    BiYamFollower and most lerobot robots). For plain @property robots, you'd
    need a different hook.
    """
    if not depth_cams:
        return robot

    for cam in depth_cams:
        if cam not in robot.cameras:
            raise KeyError(
                f"depth_cams[{cam!r}] not in robot.cameras ({list(robot.cameras)})"
            )
        cam_cfg = robot.cameras[cam].config
        if not getattr(cam_cfg, "use_depth", False):
            raise ValueError(
                f"Camera {cam!r} must be configured with use_depth=True "
                "to produce a depth stream."
            )

    original_get_observation = robot.get_observation
    original_connect = robot.connect

    def get_observation_with_depth(*args, **kwargs):
        obs = original_get_observation(*args, **kwargs)
        # Clip raw RealSense mm depth to the active range. Matches the
        # clipping the encoder applies in _quantize_depth_m, so what you
        # see in the teleop Rerun preview is exactly what gets recorded.
        # Without this, rerun would render the raw sensor output and the
        # --min/max_depth_m flags would appear to have no effect in the
        # live display.
        # RealSense uses 0 as an "invalid/no-depth" sentinel; preserve it
        # rather than clipping up to min_mm (which would fabricate a fake
        # wall at the close-range floor).
        min_mm = int(MIN_DEPTH_M * 1000)
        max_mm = int(MAX_DEPTH_M * 1000)
        for cam in depth_cams:
            depth = robot.cameras[cam].read_depth()  # uint16 mm
            valid = depth > 0
            depth = np.where(
                valid,
                np.clip(depth, min_mm, max_mm),
                0,  # keep invalid sentinel
            ).astype(np.uint16)
            if depth.ndim == 2:
                depth = depth[..., None]  # (H, W) → (H, W, 1) for shape consistency
            obs[f"{cam}_depth"] = depth
        return obs

    def _install_depth_features() -> None:
        # Must run AFTER the underlying robot's connect(), because some
        # robots (e.g. BiYamFollower) purge cached_property entries from
        # __dict__ during connect() once DOFs are known. Running before
        # connect would be silently overwritten.
        base_feats = dict(robot.observation_features)
        for cam in depth_cams:
            cam_obj = robot.cameras[cam]
            base_feats[f"{cam}_depth"] = (cam_obj.height, cam_obj.width, 1)
        robot.__dict__["observation_features"] = base_feats

    def connect_with_depth(*args, **kwargs):
        result = original_connect(*args, **kwargs)
        _install_depth_features()
        return result

    robot.get_observation = get_observation_with_depth
    robot.connect = connect_with_depth
    # Also install now in case the caller has already connected before
    # attaching depth capability.
    if getattr(robot, "is_connected", False):
        _install_depth_features()

    logger.info("Depth capture attached for cameras: %s", depth_cams)
    return robot


# ────────────────────────────────────────────────────────────────
# Feature + dataset patching
# ────────────────────────────────────────────────────────────────
def _mark_depth_features(features: dict, depth_cams: list[str]) -> dict:
    """Stamp video.is_depth_map=True onto every matching depth feature.

    Also persists the active MIN_DEPTH_M / MAX_DEPTH_M range into the info
    dict under `video.depth_min_m` / `video.depth_max_m` so that datasets
    are self-documenting — the range is otherwise NOT recoverable from the
    encoded mp4 (the quantization is applied pre-encode).
    """
    depth_suffixes = {f"{cam}_depth" for cam in depth_cams}
    out = {}
    for key, spec in features.items():
        last = key.rsplit(".", 1)[-1]
        is_video = spec.get("dtype") == "video"
        if last in depth_suffixes and is_video:
            info = dict(spec.get("info", {}))
            info["video.is_depth_map"] = True
            info.setdefault("video.codec", DEPTH_CODEC)
            info.setdefault("video.pix_fmt", DEPTH_PIX_FMT)
            info.setdefault("video.crf", DEPTH_CRF)
            info.setdefault("video.depth_min_m", MIN_DEPTH_M)
            info.setdefault("video.depth_max_m", MAX_DEPTH_M)
            info.setdefault("video.depth_q_bits", Q_BITS)
            out[key] = {**spec, "info": info}
        else:
            out[key] = spec
    return out


@contextmanager
def _inject_depth_pipeline(depth_cams: list[str]):
    """Patch LeRobotDataset.create and .resume to thread depth hooks through."""
    original_create = LeRobotDataset.create
    original_resume = LeRobotDataset.resume

    def create(*args, **kwargs):
        feats = kwargs.get("features")
        if feats is not None:
            kwargs["features"] = _mark_depth_features(feats, depth_cams)
        kwargs.setdefault("depth_map_encoding_fn", encode_depth)
        kwargs.setdefault("depth_map_decoding_fn", decode_depth)
        return original_create(*args, **kwargs)

    def resume(*args, **kwargs):
        kwargs.setdefault("depth_map_encoding_fn", encode_depth)
        kwargs.setdefault("depth_map_decoding_fn", decode_depth)
        return original_resume(*args, **kwargs)

    LeRobotDataset.create = create
    LeRobotDataset.resume = resume
    try:
        yield
    finally:
        LeRobotDataset.create = original_create
        LeRobotDataset.resume = original_resume


@contextmanager
def _patch_robot_factory(depth_cams: list[str]):
    """Wrap `make_robot_from_config` inside the record module so the robot
    returned by the lerobot_record pipeline emits depth observations.
    """
    from lerobot.scripts import lerobot_record as record_mod

    original = record_mod.make_robot_from_config

    def patched(cfg):
        robot = original(cfg)
        return _attach_depth_capability(robot, depth_cams)

    record_mod.make_robot_from_config = patched
    try:
        yield
    finally:
        record_mod.make_robot_from_config = original


# ────────────────────────────────────────────────────────────────
# Config + main
# ────────────────────────────────────────────────────────────────
@dataclass
class DepthRecordConfig(RecordConfig):
    # Cameras to ALSO capture depth from. Each must be configured with
    # use_depth=True (e.g. RealSenseCameraConfig).
    depth_cams: list[str] = field(default_factory=list)
    # Closest depth to preserve (meters). Values < this get clamped.
    min_depth_m: float = DEFAULT_MIN_DEPTH_M
    # Farthest depth to preserve (meters). Values > this get clamped.
    # For close-range manipulation tasks (0.1–1.0 m) set this ~1.0 m —
    # log-scale quantization puts more bits near the close range, so
    # narrowing gives substantially tighter per-level precision.
    max_depth_m: float = DEFAULT_MAX_DEPTH_M


@parser.wrap()
def main(cfg: DepthRecordConfig) -> LeRobotDataset:
    init_logging()
    logger.info("DepthRecordConfig:\n%s", pformat(cfg))

    if not cfg.depth_cams:
        logger.warning(
            "--depth_cams is empty; falling through to plain lerobot-record behavior."
        )
        return _record_impl(cfg)

    set_depth_range(cfg.min_depth_m, cfg.max_depth_m)
    with _patch_robot_factory(cfg.depth_cams), _inject_depth_pipeline(cfg.depth_cams):
        return _record_impl(cfg)


if __name__ == "__main__":
    main()
