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

"""Base rollout strategy: autonomous policy execution with no data recording."""

from __future__ import annotations

import gc
import logging
import time

from lerobot.utils.robot_utils import precise_sleep

from ..context import RolloutContext
from .core import RolloutStrategy, send_next_action

logger = logging.getLogger(__name__)


class BaseStrategy(RolloutStrategy):
    """Autonomous policy rollout with no data recording.

    All actions flow through the ``robot_action_processor`` pipeline
    before reaching the robot.
    """

    def setup(self, ctx: RolloutContext) -> None:
        """Initialise the inference engine."""
        self._init_engine(ctx)
        logger.info("Base strategy ready")

    def run(self, ctx: RolloutContext) -> None:
        """Run the autonomous control loop until shutdown or duration expires."""
        engine = self._engine
        cfg = ctx.runtime.cfg
        robot = ctx.hardware.robot_wrapper
        interpolator = self._interpolator

        control_interval = interpolator.get_control_interval(cfg.fps)

        start_time = time.perf_counter()
        engine.resume()
        # PROBE: disable Python GC during the loop to test whether the rare
        # ~60ms spikes in policy inference are caused by gen-2 GC pauses.
        # We trigger a manual collection every 200 iters as a safety valve.
        gc.disable()
        _gc_iter_counter = 0
        logger.warning("PROBE: Python GC DISABLED for rollout loop (manual collect every 200 iters)")
        logger.info("Base strategy control loop started")

        while not ctx.runtime.shutdown_event.is_set():
            loop_start = time.perf_counter()

            # Manual GC every 200 iters (safety valve while gc.disable() is active)
            _gc_iter_counter += 1
            if _gc_iter_counter % 200 == 0:
                _gc_t0 = time.perf_counter()
                collected = gc.collect()
                _gc_ms = (time.perf_counter() - _gc_t0) * 1e3
                if _gc_ms > 5.0:
                    logger.warning(f"PROBE: manual gc.collect() took {_gc_ms:.1f}ms ({collected} objs)")

            if cfg.duration > 0 and (time.perf_counter() - start_time) >= cfg.duration:
                logger.info("Duration limit reached (%.0fs)", cfg.duration)
                break

            obs = robot.get_observation()
            _t_after_obs = time.perf_counter()
            obs_processed = self._process_observation_and_notify(ctx.processors, obs)
            _t_after_proc = time.perf_counter()

            if self._handle_warmup(cfg.use_torch_compile, loop_start, control_interval):
                continue

            action_dict = send_next_action(obs_processed, obs, ctx, interpolator)
            _t_after_act = time.perf_counter()
            self._log_telemetry(obs_processed, action_dict, ctx.runtime)

            dt = time.perf_counter() - loop_start
            if (sleep_t := control_interval - dt) > 0:
                precise_sleep(sleep_t)
            else:
                # PROBE: full breakdown when iteration is severely slow (>50ms)
                obs_ms = (_t_after_obs - loop_start) * 1e3
                proc_ms = (_t_after_proc - _t_after_obs) * 1e3
                act_ms = (_t_after_act - _t_after_proc) * 1e3
                tail_ms = (time.perf_counter() - _t_after_act) * 1e3
                total_ms = dt * 1e3
                if total_ms > 40.0:
                    inner = getattr(robot, "last_obs_timings", None)
                    inner_str = ""
                    if inner:
                        cam_str = " ".join(f"{k}={v:.1f}" for k, v in inner["cam_per"].items())
                        inner_str = (
                            f" | get_obs[left_rpc={inner['left_rpc_ms']:.1f} "
                            f"right_rpc={inner['right_rpc_ms']:.1f} "
                            f"cams={inner['cams_ms']:.1f} ({cam_str})]"
                        )
                    act_inner = getattr(send_next_action, "last_timings", None)
                    act_str = ""
                    if act_inner:
                        act_str = (
                            f" | send_act[infer={act_inner['infer_ms']:.1f} "
                            f"cpu_sync={act_inner['cpu_sync_ms']:.1f} "
                            f"send_rpc={act_inner['send_ms']:.1f} "
                            f"ran_infer={act_inner['ran_inference']}]"
                        )
                    logger.warning(
                        f"SLOW ITER {1 / dt:.1f}Hz total={total_ms:.1f}ms: "
                        f"get_obs={obs_ms:.1f} proc={proc_ms:.1f} send_action={act_ms:.1f} "
                        f"tail={tail_ms:.1f}{inner_str}{act_str}"
                    )
                else:
                    logger.warning(
                        f"Record loop is running slower ({1 / dt:.1f} Hz) than the target FPS ({cfg.fps} Hz). Dataset frames might be dropped and robot control might be unstable. Common causes are: 1) Camera FPS not keeping up 2) Policy inference taking too long 3) CPU starvation"
                    )

    def teardown(self, ctx: RolloutContext) -> None:
        """Disconnect hardware and stop inference."""
        self._teardown_hardware(
            ctx.hardware,
            return_to_initial_position=ctx.runtime.cfg.return_to_initial_position,
        )
        logger.info("Base strategy teardown complete")
