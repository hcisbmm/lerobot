#!/usr/bin/env bash
set -euo pipefail

CAMERAS='{"left": {"type": "opencv", "index_or_path": 16, "width": 640, "height": 480, "fps": 30, "fourcc": "YUY2"}, "right": {"type": "opencv", "index_or_path": 10, "width": 640, "height": 480, "fps": 30, "fourcc": "YUY2"}, "front": {"type": "opencv", "index_or_path": 24, "width": 640, "height": 480, "fps": 30, "fourcc": "YUY2"}}'

uv run lerobot-rollout \
    --robot.type=bi_yam_follower \
    --robot.left_arm_port=1235 \
    --robot.right_arm_port=1234 \
    --teleop.type=bi_yam_leader \
    --teleop.left_arm_port=5002 \
    --teleop.right_arm_port=5001 \
    --robot.cameras="${CAMERAS}" \
    --strategy.type=dagger \
    --strategy.record_autonomous=false \
    --dataset.repo_id=hcisbmm/rollout_dagger_vial_insertion_03_act_front_bimanual \
    --policy.path=hcisbmm/vial_insertion_03_act_front_bimanual \
    --task="Pick up the blue-cap vial on the desk, and insert it into the hole of the rack." \
    --duration=999 \
    --display_data=true \
    --use_torch_compile=true \
    --policy.n_action_steps=10
    # ── Alternative for completely smooth motion (slower loop ~13 Hz):
    #   --policy.temporal_ensemble_coeff=0.01 \
    #   --policy.n_action_steps=1
    # See commit message for context: chunk-boundary discontinuity diagnosis.
