"""Microbenchmark prepare_observation_for_inference + a CPU→GPU alternative.
Measures the actual cost of the per-tick observation preprocessing for a 3-cam
640x480 RGB observation, which the rollout does on every tick regardless of
whether the policy will run inference or popleft a cached action.
"""
import time
import numpy as np
import torch
from lerobot.policies.utils import prepare_observation_for_inference

DEVICE = torch.device("cuda")
N = 100


def make_obs():
    return {
        "observation.images.left":  np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8),
        "observation.images.right": np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8),
        "observation.images.front": np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8),
        "observation.state":        np.random.rand(14).astype(np.float32),
    }


def prep_current(obs):
    return prepare_observation_for_inference(dict(obs), DEVICE, "task", "robot")


def prep_alt(obs):
    """Alternative: send uint8 to GPU first, do cast/permute on GPU."""
    out = {}
    for name, arr in obs.items():
        t = torch.from_numpy(arr).to(DEVICE, non_blocking=True)
        if "image" in name:
            t = t.float().div_(255).permute(2, 0, 1).contiguous()
        out[name] = t.unsqueeze(0)
    out["task"] = "task"
    out["robot_type"] = "robot"
    return out


def bench(label, fn, n=N):
    obs = make_obs()
    # Warm
    for _ in range(5):
        fn(obs)
    torch.cuda.synchronize()
    times = []
    for _ in range(n):
        torch.cuda.synchronize()
        t0 = time.perf_counter()
        fn(obs)
        torch.cuda.synchronize()
        times.append((time.perf_counter() - t0) * 1e3)
    times = np.array(times)
    print(f"{label:25s}  mean={times.mean():6.2f}ms  median={np.median(times):6.2f}  p95={np.percentile(times,95):6.2f}  min={times.min():.2f}  max={times.max():.2f}")


print(f"torch={torch.__version__}, cuda={torch.cuda.is_available()}")
print(f"observation: 3x (480, 640, 3) uint8 + 14 float32, target device={DEVICE}\n")
bench("prep_current (lerobot)", prep_current)
bench("prep_alt   (uint8→GPU)", prep_alt)
