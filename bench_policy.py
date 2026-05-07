"""Minimal benchmark of ACT policy inference. Bypasses the rollout loop / robot /
cameras to isolate compile + ensembler behavior.
"""
import os
import sys
import time
import numpy as np
import torch

# Match the lerobot rollout setup
from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.factory import get_policy_class
from lerobot.policies.utils import prepare_observation_for_inference

REPO = "hcisbmm/vial_insertion_03_act_front_bimanual"
DEVICE = torch.device("cuda")
N_WARMUP = 5
N_ITERS = 50


def make_fake_obs():
    return {
        "observation.images.left":  np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8),
        "observation.images.right": np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8),
        "observation.images.front": np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8),
        "observation.state":        np.random.rand(14).astype(np.float32),
    }


def main():
    use_compile = os.environ.get("USE_COMPILE", "1") == "1"
    print(f"[bench] use_compile={use_compile} torch={torch.__version__}")

    cfg = PreTrainedConfig.from_pretrained(REPO)
    policy_class = get_policy_class(cfg.type)
    policy = policy_class.from_pretrained(REPO, config=cfg)
    policy = policy.to(DEVICE)
    policy.eval()
    policy.reset()

    print(f"[bench] policy={cfg.type} temporal_ensemble_coeff={cfg.temporal_ensemble_coeff}")
    print(f"[bench] chunk_size={cfg.chunk_size} n_action_steps={cfg.n_action_steps}")

    if use_compile:
        policy.predict_action_chunk = torch.compile(
            policy.predict_action_chunk, backend="inductor", mode="default"
        )

    # Warmup (triggers compile on first call if enabled)
    print(f"[bench] warming up ({N_WARMUP} iters)...")
    for i in range(N_WARMUP):
        obs = make_fake_obs()
        obs = prepare_observation_for_inference(obs, DEVICE, task="bench", robot_type="bi_yam_follower")
        t0 = time.perf_counter()
        with torch.inference_mode():
            _ = policy.select_action(obs)
        torch.cuda.synchronize()
        print(f"  warmup {i}: {(time.perf_counter() - t0) * 1e3:7.1f} ms")

    # Steady-state measurement
    print(f"[bench] measuring ({N_ITERS} iters)...")
    times = []
    fwd_times = []
    for i in range(N_ITERS):
        obs = make_fake_obs()
        obs = prepare_observation_for_inference(obs, DEVICE, task="bench", robot_type="bi_yam_follower")

        torch.cuda.synchronize()
        t0 = time.perf_counter()
        with torch.inference_mode():
            # Time the inner predict_action_chunk separately
            t_fwd_start = time.perf_counter()
            _chunk = policy.predict_action_chunk({k: v for k, v in obs.items() if k != "task" and k != "robot_type"})
            torch.cuda.synchronize()
            t_fwd = time.perf_counter() - t_fwd_start

            # Then full select_action (which calls predict_action_chunk again — that's the policy logic)
            _ = policy.select_action(obs)
        torch.cuda.synchronize()
        dt = time.perf_counter() - t0
        times.append(dt * 1e3)
        fwd_times.append(t_fwd * 1e3)
        if i < 10 or i % 5 == 0:
            print(f"  iter {i:3d}: select={dt*1e3:7.1f} ms  predict_action_chunk={t_fwd*1e3:7.1f} ms")

    times = np.array(times)
    fwd_times = np.array(fwd_times)
    print()
    print(f"[bench] select_action       (n={len(times)}): "
          f"mean={times.mean():.1f} ms  median={np.median(times):.1f} ms  "
          f"p95={np.percentile(times,95):.1f} ms  min={times.min():.1f}  max={times.max():.1f}")
    print(f"[bench] predict_action_chunk (n={len(fwd_times)}): "
          f"mean={fwd_times.mean():.1f} ms  median={np.median(fwd_times):.1f} ms  "
          f"p95={np.percentile(fwd_times,95):.1f} ms  min={fwd_times.min():.1f}  max={fwd_times.max():.1f}")


if __name__ == "__main__":
    sys.exit(main() or 0)
