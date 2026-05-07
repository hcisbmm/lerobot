#!/usr/bin/env bash
# Async-inference rollout launcher: spawns policy_server + robot_client as
# subprocesses. Mirrors the configuration of run_rollout.sh.
#
# Server config: minimal — the policy itself is uploaded by the client at
# connect time (see RemotePolicyConfig in policy_server.py:125).
# Client config: matches run_rollout.sh's robot/camera/policy settings.

set -euo pipefail

# --- Tunables -----------------------------------------------------------------
HOST="localhost"
PORT="8080"
FPS=30
POLICY_REPO="hcisbmm/vial_insertion_03_act_front_bimanual"
TASK="Pick up the blue-cap vial on the desk, and insert it into the hole of the rack."

# Async-specific knobs (see RobotClientConfig in async_inference/configs.py)
ACTIONS_PER_CHUNK=50            # how many of ACT's chunk_size to actually send back
CHUNK_SIZE_THRESHOLD=0.5        # request next chunk when half the queue is consumed
AGGREGATE_FN="weighted_average" # how to merge overlapping chunks (smoothing analog of temporal ensembling)

CAMERAS='{"left": {"type": "opencv", "index_or_path": 16, "width": 640, "height": 480, "fps": 30, "fourcc": "YUY2"}, "right": {"type": "opencv", "index_or_path": 10, "width": 640, "height": 480, "fps": 30, "fourcc": "YUY2"}, "front": {"type": "opencv", "index_or_path": 24, "width": 640, "height": 480, "fps": 30, "fourcc": "YUY2"}}'

# --- Lifecycle ----------------------------------------------------------------
SERVER_PID=""
cleanup() {
    local exit_code=$?
    if [[ -n "${SERVER_PID}" ]] && kill -0 "${SERVER_PID}" 2>/dev/null; then
        echo "[launcher] Stopping policy_server (pid=${SERVER_PID})..."
        kill -TERM "${SERVER_PID}" 2>/dev/null || true
        # Give it a moment to flush logs / release the port
        for _ in {1..20}; do
            kill -0 "${SERVER_PID}" 2>/dev/null || break
            sleep 0.1
        done
        kill -KILL "${SERVER_PID}" 2>/dev/null || true
    fi
    exit "${exit_code}"
}
trap cleanup INT TERM EXIT

wait_for_port() {
    local host="$1" port="$2" timeout_s="${3:-30}"
    echo "[launcher] Waiting for ${host}:${port} (timeout ${timeout_s}s)..."
    local start=$SECONDS
    while ! (exec 3<>"/dev/tcp/${host}/${port}") 2>/dev/null; do
        if (( SECONDS - start > timeout_s )); then
            echo "[launcher] Server did not open ${host}:${port} within ${timeout_s}s" >&2
            return 1
        fi
        sleep 0.2
    done
    exec 3<&-
    exec 3>&-
    echo "[launcher] ${host}:${port} is up."
}

# --- 1. Spawn server ----------------------------------------------------------
echo "[launcher] Starting policy_server on ${HOST}:${PORT}..."
uv run python -m lerobot.async_inference.policy_server \
    --host="${HOST}" \
    --port="${PORT}" \
    --fps="${FPS}" &
SERVER_PID=$!
echo "[launcher] policy_server pid=${SERVER_PID}"

wait_for_port "${HOST}" "${PORT}" 60

# --- 2. Run client (foreground) ----------------------------------------------
echo "[launcher] Starting robot_client → ${HOST}:${PORT}..."
uv run python -m lerobot.async_inference.robot_client \
    --server_address="${HOST}:${PORT}" \
    --policy_type=act \
    --pretrained_name_or_path="${POLICY_REPO}" \
    --policy_device=cuda \
    --client_device=cpu \
    --robot.type=bi_yam_follower \
    --robot.left_arm_port=1235 \
    --robot.right_arm_port=1234 \
    --robot.cameras="${CAMERAS}" \
    --task="${TASK}" \
    --fps="${FPS}" \
    --actions_per_chunk="${ACTIONS_PER_CHUNK}" \
    --chunk_size_threshold="${CHUNK_SIZE_THRESHOLD}" \
    --aggregate_fn_name="${AGGREGATE_FN}"

# Trap will tear down the server on any exit path.
