# Bimanual Yam Arms with LeRobot

This guide explains how to use bimanual Yam arms with LeRobot for data collection.

## Overview

The bimanual Yam setup consists of:

- **2 Follower Arms**: Controlled by LeRobot to execute actions
- **2 Leader Arms**: With teaching handles for teleoperation
- **4 CAN Interfaces**: For communication with the arms
- **XELA tactile pad (optional)**: A 4×4 magnetic-field tactile sensor on the
  right-side fingertip of the right gripper, served by `xela_server` over a
  local WebSocket. See [Tactile Sensor (XELA, optional)](#tactile-sensor-xela-optional).

## Hardware Setup

### Required CAN Interfaces

You need to set up 4 CAN interfaces with the following names:

- `can_follower_r`: Right follower arm
- `can_follower_l`: Left follower arm
- `can_leader_r`: Right leader arm (with teaching handle)
- `can_leader_l`: Left leader arm (with teaching handle)

### CAN Interface Configuration

For details on setting up persistent CAN interface names, see:

- `i2rt/doc/set_persist_id_socket_can.md`

Make sure all CAN interfaces are UP and accessible:

```bash
ip link show can_follower_r
ip link show can_follower_l
ip link show can_leader_r
ip link show can_leader_l
```

### Initialize CAN Interfaces

Before starting the Yam arm servers, you need to initialize all CAN interfaces with the correct bitrate (1000000).

A Python script is provided to automatically detect and reset all CAN interfaces:

```bash
python src/lerobot/robots/bi_yam_follower/reset_can_interfaces.py
```

This script will:

- Detect all CAN interfaces on your system
- Reset each interface with bitrate 1000000
- Provide clear feedback about the configuration status

**Alternative: Manual reset with bash**

If you prefer a bash script, you can create this file and run it:

```bash
#!/bin/bash

if [ "$(id -u)" != "0" ]; then
    SUDO="sudo"
else
    SUDO=""
fi

# Function to reset a CAN interface
reset_can_interface() {
    local iface=$1
    echo "Resetting CAN interface: $iface"
    $SUDO ip link set "$iface" down
    $SUDO ip link set "$iface" up type can bitrate 1000000
}

# Get all CAN interfaces
can_interfaces=$(ip link show | grep -oP '(?<=: )(can\w+)')

# Check if any CAN interfaces were found
if [[ -z "$can_interfaces" ]]; then
    echo "No CAN interfaces found."
    exit 1
fi

# Reset each CAN interface
echo "Detected CAN interfaces: $can_interfaces"
for iface in $can_interfaces; do
    reset_can_interface "$iface"
done

echo "All CAN interfaces have been reset with bitrate 1000000."
```

Save as `reset_can.sh`, make it executable with `chmod +x reset_can.sh`, and run with `./reset_can.sh`.

### Optional: XELA Tactile Sensor

To capture fingertip tactile readings during recording, mount an XELA XR1944
(4×4 taxels × 3 axes) pad on the right-side fingertip of the right gripper.
The sensor connects to the host via a **VScom USB-CAN Plus** adapter
(`/dev/ttyUSB0`) with **two USB cables — both must be connected** (one is
sensor-side power). The slCAN bus this exposes is independent of the 4 arm
CAN interfaces above. The vendor software `xela_server` reads the slCAN
stream and serves the latest reading on a local WebSocket (port 5000).

For the one-time vendor software install (apt `can-utils`, `/etc/xela`
directory + 777 perms, unpack `appimage.zip`, `PATH` setup, interactive
`xela_conf`), see [**First-time setup**](../../tactile/xela/README.md#first-time-setup-one-time-per-machine)
in the XELA backend README.

For per-boot bring-up and per-session usage, see the
[Tactile Sensor (XELA, optional)](#tactile-sensor-xela-optional)
section below. No additional Python dependencies are needed —
`websocket-client` is pulled in transitively by the base install.

## Software Setup

### Platform Support

**Note:** Bimanual Yam arms require Linux for hardware operation due to:

- CAN interface support (SocketCAN on Linux)
- Hardware control libraries designed for Linux

### Install Dependencies

**Install LeRobot with Yam support:**

For Yam arms with Intel RealSense camera support (recommended):

```bash
pip install -e ".[yam,intelrealsense]"
```

For basic Yam arm functionality with OpenCV cameras:

```bash
pip install -e ".[yam]"
```

This will install:

- `portal` - RPC framework for client-server communication (Linux only)
- `i2rt` - Robotics library providing the server infrastructure ([i2rt-robotics/i2rt](https://github.com/i2rt-robotics/i2rt))
- `pyrealsense2` - Intel RealSense camera support (if using intelrealsense extra)

**Verify installation:**

```bash
python -c "import i2rt, portal; print('Dependencies OK')"
```

## Running the System

### Step 1: Start the Unified Server

The easiest way to start all 4 arm servers is using the unified server script. Friction-compensation defaults are loaded from the arm + gripper YAMLs at launch (see [Friction compensation flags](#friction-compensation-flags-overrides) below), so the bare command below already applies the saved per-arm tuning:

```bash
python src/lerobot/robots/bi_yam_follower/run_bimanual_yam_server.py
```

For one-off friction-comp tuning experiments without editing the YAML, override on the CLI:

```bash
python src/lerobot/robots/bi_yam_follower/run_bimanual_yam_server.py \
  --enable_friction_comp True \
  --friction_breakaway 1.2 1.8 1.6 0.1 0 0 0 \
  --friction_eps 0.005
```

Once you find values that work, copy them into the YAMLs (see [Where to store the tuned values](#where-to-store-the-tuned-values)) so future launches don't need any flags.

This single command starts all 4 servers (2 followers + 2 leaders) with default settings:

- Right follower: port 1234 (CAN: `can_follower_r`)
- Left follower: port 1235 (CAN: `can_follower_l`)
- Right leader: port 5001 (CAN: `can_leader_r`)
- Left leader: port 5002 (CAN: `can_leader_l`)

**Customize CAN interfaces and ports:**

```bash
python src/lerobot/robots/bi_yam_follower/run_bimanual_yam_server.py \
  --right_follower_can can0 \
  --left_follower_can can1 \
  --right_leader_can can2 \
  --left_leader_can can3
```

#### Friction compensation flags (overrides)

Coulomb friction feedforward (Cff in MIT mode) helps the first few joints overcome static friction on small leader motions without raising `kp`. Per-arm defaults live in the YAML configs under `third_party/i2rt/i2rt/robots/config/`, so **you don't need any flag for normal operation** — they're applied automatically based on `--follower_arm_type` (default `yam_ultra`). Use the flags below only to override on the fly.

| Flag                     | Type                  | Default           | What it does                                                                                                                                                                                                                                                            |
| ------------------------ | --------------------- | ----------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `--enable_friction_comp` | `{None, True, False}` | `None` → use YAML | Force enable / force disable. `None` (omit the flag) means "honor the YAML's `friction_comp.enable`".                                                                                                                                                                   |
| `--friction_breakaway`   | list of floats        | `()` → use YAML   | Per-joint breakaway torque (Nm) — the saturation amplitude of the smooth-tanh feedforward. Length **6** = arm joints only (gripper element is appended from the gripper YAML); length **7** = full arm + gripper override. Set 0 for joints you don't want compensated. |
| `--friction_eps`         | float                 | `None` → use YAML | Saturation width (rad) for the tanh blend (`τ = breakaway · tanh(err / eps)`). Smaller `eps` = sharper, more aggressive transition; larger = smoother but less stiction relief. Typical range 0.003–0.02.                                                               |

Tuning loop: launch with the bare command, hand-drive the followers, watch for joints that lag or stick on small motions, then re-launch with overrides like `--friction_breakaway 1.2 1.8 1.6 0.1 0 0 0 --friction_eps 0.005` and iterate. Increase `breakaway` for sticky joints, lower `eps` if motion feels mushy, raise `eps` if joints chatter at zero crossings.

#### Where to store the tuned values

Once you've found values that work, save them to the YAMLs so every future launch picks them up without any flags. For the default `yam_ultra` follower with a `linear_4310` (a.k.a. `v3`) gripper:

- **Arm YAML** — `third_party/i2rt/i2rt/robots/config/yam_ultra.yml` (or `yam.yml` / `yam_pro.yml` / `big_yam.yml` if you use a different `--follower_arm_type`):

  ```yaml
  friction_comp:
    enable: true
    breakaway: [1.2, 1.8, 1.6, 0.1, 0.0, 0.0] # 6 = arm joints only (J0..J5)
    eps: 0.005
  ```

- **Gripper YAML** — `third_party/i2rt/i2rt/robots/config/linear_4310.yml` (or whichever gripper you use):

  ```yaml
  friction_comp_breakaway: 0.0 # set non-zero only if the gripper motor itself benefits from comp
  ```

At runtime `get_yam_robot()` concatenates `arm.friction_comp.breakaway` (length 6) with the gripper's `friction_comp_breakaway` (length 1) into the final length-7 array, mirroring how `kp` / `kd` are assembled.

**Run follower-only mode (without teaching handles):**

```bash
python src/lerobot/robots/bi_yam_follower/run_bimanual_yam_server.py \
  --mode follower_only
```

Leave this terminal running while recording data.

**Optional — start `xela_server` for tactile recording:**

If your recording will include the XELA tactile pad, start `xela_server` in
a separate terminal alongside the arm servers. After the one-time slcand
setup (see [Tactile Sensor (XELA, optional)](#tactile-sensor-xela-optional)),
each session is just:

```bash
python src/lerobot/robots/bi_yam_follower/run_xela_server.py
```

Leave both this terminal AND the arm-server terminal running while recording.

### Step 2: Testing and Setup

#### Step 2.1: Test Teleoperator (In another terminal)

Before recording, test that the teleoperator connection works (--compare_leader_follower_data to monitor the difference btw leader and follower joint position):

```bash
lerobot-teleoperate \
  --robot.type=bi_yam_follower \
  --robot.left_arm_port=1235 \
  --robot.right_arm_port=1234 \
  --teleop.type=bi_yam_leader \
  --teleop.left_arm_port=5002 \
  --teleop.right_arm_port=5001 \
  --display_data=true \
  --compare_leader_follower_data=left
```

#### With Torque Input

```bash
lerobot-teleoperate \
  --robot.type=bi_yam_follower \
  --robot.left_arm_port=1235 \
  --robot.right_arm_port=1234 \
  --teleop.type=bi_yam_leader \
  --teleop.left_arm_port=5002 \
  --teleop.right_arm_port=5001 \
  --display_data=true \
  --record_effort=true
```

#### With Live 3D MuJoCo Torque + EE-Force Visualizer

Opens a MuJoCo viewer window with the two YAM arms mirroring the follower's
live pose. Draws an arrow along each joint axis sized and colored by the
measured joint torque (red = positive, blue = negative), and overlays a green
arrow at each gripper site representing the estimated end-effector force
(`F_ee ≈ pinv(J^T) · (τ_meas − g(q))`). With gravity compensation on
(default), the force arrows should be near-zero when the arm is held still
under gravity alone and should point along the direction of any external
push.

```bash
lerobot-teleoperate \
  --robot.type=bi_yam_follower \
  --robot.left_arm_port=1235 \
  --robot.right_arm_port=1234 \
  --teleop.type=bi_yam_leader \
  --teleop.left_arm_port=5002 \
  --teleop.right_arm_port=5001 \
  --display_data=true \
  --show_mujoco_torque=true
```

Useful flags (defaults shown):

- `--show_mujoco_torque=true` — enable the 3D viewer.
- `--mujoco_gripper_type=linear_4310` — gripper MJCF variant used to compose
  the scene. Change if your follower uses a different i2rt gripper.
- `--mujoco_show_ee_force=true` — overlay per-gripper force arrows.
- `--mujoco_compensate_gravity=true` — subtract `g(q)` before the Jacobian
  inversion. Set to `false` for a raw-vs-compensated sanity check.
- `--mujoco_arrow_scale=0.05` — metres per Nm for joint-torque arrows.
- `--mujoco_force_arrow_scale=0.01` — metres per Newton for EE-force arrows.
- `--mujoco_max_ee_force=50.0` — clamp on `|F_ee|` (Newtons) to keep the
  overlay readable near arm singularities.

Physics caveats: `.eff` is already in N·m from the DM-motor firmware (no K_t
conversion in our code), so readings reflect motor-side friction but not
cable/joint-side friction. The EE force is a quasi-static estimate;
unmodeled dynamics and a mismatch between the real arm's end-effector mass
and the MJCF mass will leak into the arrow. Near arm singularities the
pseudoinverse blows up — `--mujoco_max_ee_force` clamps it.

#### Step 2.2: Find Camera

Identify available cameras on your system:

```bash
lerobot-find-cameras realsense
```

or for OpenCV cameras:

```bash
lerobot-find-cameras opencv
```

For the following steps, adjust the camera `index_or_path` based on the output from this command.

#### Palm Camera Setup

The default config includes two palm cameras (`left_palm`, `right_palm`) using OpenCV. Before using them, you need to set their stable device paths:

1. **Find stable device paths** (persistent across reboots):

```bash
ls -la /dev/v4l/by-id/
```

This shows symlinks like:

```
usb-<Manufacturer>_<Model>_<Serial>-video-index0 -> ../../video21
```

2. **Update the config** in `config_bi_yam_follower.py` — replace the placeholder `index_or_path` values with your actual paths:

```python
"left_palm": OpenCVCameraConfig(
    index_or_path="/dev/v4l/by-id/usb-YOUR_LEFT_CAMERA-video-index0",
    fps=30, width=640, height=480,
    rotation=Cv2Rotation.NO_ROTATION,
),
```

3. **Rotation options** — if your camera image is sideways or upside down, change the `rotation` parameter:
   - `Cv2Rotation.NO_ROTATION` (0°)
   - `Cv2Rotation.ROTATE_90` (90° clockwise)
   - `Cv2Rotation.ROTATE_180` (180°)
   - `Cv2Rotation.ROTATE_270` (270° clockwise / 90° counter-clockwise)

   Note: `width` and `height` in the config represent the **post-rotation** output dimensions.

#### Step 2.3: Test Camera + Teleoperator

Note: Replace the `index_or_path` values with the camera indices found in the previous step. If you have different cameras or want to use different settings, adjust the camera configuration accordingly.

**Using default palm cameras (configured in `config_bi_yam_follower.py`):**

```bash
lerobot-teleoperate \
  --robot.type=bi_yam_follower \
  --teleop.type=bi_yam_leader \
  --display_data=true
```

Palm camera frames will appear in the Rerun viewer as `observation.left_palm` and `observation.right_palm`.

**Using palm cameras + RealSense top camera (CLI override):**

```bash
lerobot-teleoperate \
  --robot.type=bi_yam_follower \
  --teleop.type=bi_yam_leader \
  --display_data=true \
  --robot.cameras='{
    top: {"type": "intelrealsense", "serial_number_or_name": "141722076304", "width": 640, "height": 480, "fps": 30},
    left: {"type": "intelrealsense", "serial_number_or_name": "335122271633", "width": 640, "height": 480, "fps": 30},
    right: {"type": "intelrealsense", "serial_number_or_name": "323622271837", "width": 640, "height": 480, "fps": 30}
  }' \
  --robot.use_palm_camera=true \
  --robot.palm_camera_auto_exposure=1 \
  --robot.palm_camera_exposure=200

```

**Palm camera parameters**

| Flag                                | Default | Description                                                                                                                                                                                                                   |
| ----------------------------------- | ------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `--robot.use_palm_camera`           | `false` | Enable the two palm USB cameras (`left_palm`, `right_palm`). Their device paths are pinned via `/dev/v4l/by-path/...` in [config_bi_yam_follower.py](config_bi_yam_follower.py).                                              |
| `--robot.palm_camera_fps`           | `30`    | Frame rate for both palm cameras.                                                                                                                                                                                             |
| `--robot.palm_camera_fourcc`        | `MJPG`  | FOURCC codec. `MJPG` is required to hit 30 fps on most USB 2.0 webcams; set to `None` to auto-detect.                                                                                                                         |
| `--robot.palm_camera_auto_exposure` | `None`  | `cv2.CAP_PROP_AUTO_EXPOSURE`. On V4L2: `1` = manual (locked exposure), `3` = aperture priority (auto). Leave unset to keep the driver default.                                                                                |
| `--robot.palm_camera_exposure`      | `None`  | `cv2.CAP_PROP_EXPOSURE`. Only applied when `auto_exposure=1`. V4L2 unit ≈ 100 µs (so `200` ≈ 20 ms shutter). Typical Arducam range: `3–2047`. Higher = brighter but more motion blur and lower max FPS. Recommended value 200 |

To find the valid exposure range for your camera:

```bash
v4l2-ctl -d /dev/v4l/by-path/pci-0000:00:14.0-usb-0:9.3.4:1.0-video-index0 -L | grep -A1 exposure
```

Setting `auto_exposure=1` + a fixed `exposure` value avoids the 1–2 s overexposure that the auto-exposure algorithm produces when the gripper opens/closes.

**Using Intel RealSense cameras:**

```bash
lerobot-teleoperate \
  --robot.type=bi_yam_follower \
  --robot.left_arm_port=1235 \
  --robot.right_arm_port=1234 \
  --teleop.type=bi_yam_leader \
  --teleop.left_arm_port=5002 \
  --teleop.right_arm_port=5001 \
  --display_data=true \
  --display_compressed_images=true \
  --fps=30 \
  --robot.cameras='{
    top: {"type": "intelrealsense", "serial_number_or_name": "141722076304", "width": 640, "height": 480, "fps": 30},
    left: {"type": "intelrealsense", "serial_number_or_name": "335122271633", "width": 640, "height": 480, "fps": 30},
    right: {"type": "intelrealsense", "serial_number_or_name": "323622271837", "width": 640, "height": 480, "fps": 30}
  }'
```

**Note:** The top camera (D435, serial 141722076304) is configured at 640x480 instead of 1280x720 because the D435 model does not support 1280x720 at 30fps. All cameras are kept at 30fps for synchronized data collection. The left and right cameras (D405 models) support the full 1280x720@30fps.

**Teleoperate with live depth stream (RealSense):**

Use the custom `lerobot-teleoperate-with-depth` entry point to verify depth
sensor output without starting a recording session. It runs the standard
teleop loop but also reads the depth stream from every camera listed in
`--depth_cams`, exposing it to Rerun (and any downstream consumer) as an
additional `{cam}_depth` observation key. Every listed camera must be
configured with `use_depth=true`.

```bash
lerobot-teleoperate-with-depth \
  --robot.type=bi_yam_follower \
  --robot.left_arm_port=1235 \
  --robot.right_arm_port=1234 \
  --robot.cameras='{
    top:   {"type": "intelrealsense", "serial_number_or_name": "141722076304", "width": 640,  "height": 480, "fps": 30, "use_depth": true},
    left:  {"type": "intelrealsense", "serial_number_or_name": "335122271633", "width": 640, "height": 480, "fps": 30, "use_depth": true},
    right: {"type": "intelrealsense", "serial_number_or_name": "323622271837", "width": 640, "height": 480, "fps": 30, "use_depth": true}
  }' \
  --teleop.type=bi_yam_leader \
  --teleop.left_arm_port=5002 \
  --teleop.right_arm_port=5001 \
  --display_data=true \
  --fps=30 \
  --depth_cams='[top, left, right]' \
  --min_depth_m=0.10 \
  --max_depth_m=1.00
```

The `--min_depth_m` / `--max_depth_m` flags behave the same as in
`lerobot-record-with-depth` — narrow to your working volume for tighter
precision. Recommended values match what you plan to record with.

Useful for:

- validating RealSense depth alignment and noise behavior,
- tuning `MIN_DEPTH_M` / `MAX_DEPTH_M` in
  `src/lerobot/scripts/lerobot_record_with_depth.py` before recording,
- checking that all three depth streams are alive before committing to a
  multi-episode dataset.

**Throughput note (measured on this machine):** with 3 RealSenses at the
resolutions above AND depth enabled on all three AND `--display_data=true`,
the teleop loop runs at ~12-20 Hz (median ~15 Hz) rather than the 30 Hz
target because each frame now carries ~3.7 MB of extra depth data per D405
plus the Rerun serialization load. Options if you need higher rates:

- Drop the D405s to `848x480` (standard use with depth; cuts throughput ~2.7×).
- Use `--display_data=false` — Rerun is the biggest single cost after cameras.
- Lower the target fps explicitly (`--fps=15`) — matches reality and removes
  sleep churn from the loop warning.

Depth is only supported for cameras whose backend exposes a `read_depth()`
method — currently that means Intel RealSense. OpenCV / palm USB webcams
will not produce a depth stream.

**Using OpenCV cameras:**

```bash
lerobot-teleoperate \
  --robot.type=bi_yam_follower \
  --robot.left_arm_port=1235 \
  --robot.right_arm_port=1234 \
  --teleop.type=bi_yam_leader \
  --teleop.left_arm_port=5002 \
  --teleop.right_arm_port=5001 \
  --display_data=true \
  --fps=30 \
  --robot.cameras='{
    top: {"type": "intelrealsense", "serial_number_or_name": "141722076304", "width": 640, "height": 480, "fps": 30},
    left: {"type": "opencv", "index_or_path": 2, "width": 640, "height": 480, "fps": 30},
    right: {"type": "opencv", "index_or_path": 10, "width": 640, "height": 480, "fps": 30}
  }'
```

#### Step 2.4: Test Tactile Sensor (optional)

If you've mounted the XELA pad, verify it connects before recording. The
simplest test is to launch the teleoperate flow with the sensor configured:
the connection is logged on startup, confirming `xela_server` is reachable
at the resolved host:port.

> **Prerequisite:** `xela_server` must already be running. Start it as
> described in the [Tactile Sensor (XELA, optional)](#tactile-sensor-xela-optional)
> section.

```bash
lerobot-teleoperate \
  --robot.type=bi_yam_follower \
  --robot.tactile_sensors='{right_finger_r: {type: xela, port: 5000, sensor_id: "1", model: XR1944}}' \
  --teleop.type=bi_yam_leader \
  --display_data=true
```

You should see two log lines confirming the sensor came up:

```
INFO ... _tactile.py:161 XELA host='auto' resolved to <your.local.ip>
INFO ... _tactile.py:182 XELA WS connected (<host>:5000)
```

If either line is missing, or the WebSocket times out, see
[Troubleshooting → Tactile Sensor Issues](#tactile-sensor-issues).

#### Step 2.5: Login to HuggingFace

Before recording, log in to HuggingFace:

```bash
uvx hf auth login
```

### Step 3: Record Data with LeRobot

In a new terminal, use `lerobot-record` to collect data.

**With default palm cameras:**

```bash
lerobot-record \
  --robot.type=bi_yam_follower \
  --teleop.type=bi_yam_leader \
  --dataset.repo_id="${HF_USER}/bimanual-yam-palm-demo" \
  --dataset.num_episodes=10 \
  --dataset.single_task="Pick and place the object" \
  --display_data=true \
  --dataset.fps=30 \
  --dataset.vcodec=hevc_nvenc \
  --dataset.streaming_encoding=true \
  --dataset.encoder_threads=2
```

**With RealSense cameras (original setup):**

```bash
lerobot-record \
  --robot.type=bi_yam_follower \
  --robot.left_arm_port=1235 \
  --robot.right_arm_port=1234 \
  --robot.cameras='{
    top: {"type": "intelrealsense", "serial_number_or_name": "141722076304", "width": 640, "height": 480, "fps": 30},
    left: {"type": "intelrealsense", "serial_number_or_name": "335122271633", "width": 640, "height": 480, "fps": 30},
    right: {"type": "intelrealsense", "serial_number_or_name": "323622271837", "width": 640, "height": 480, "fps": 30}
  }' \
  --teleop.type=bi_yam_leader \
  --teleop.left_arm_port=5002 \
  --teleop.right_arm_port=5001 \
  --dataset.repo_id="HGLLL/bimanual-yam-demo" \
  --dataset.num_episodes=10 \
  --dataset.reset_time_s=25 \
  --dataset.single_task="Pick and place the object" \
  --display_data=true \
  --dataset.fps=30 \
  --dataset.vcodec=hevc_nvenc \
  --dataset.streaming_encoding=true \
  --dataset.encoder_threads=2
```

#### With Torque Recording

```bash
lerobot-record \
  --robot.type=bi_yam_follower \
  --robot.left_arm_port=1235 \
  --robot.right_arm_port=1234 \
  --robot.cameras='{
    top: {"type": "intelrealsense", "serial_number_or_name": "141722076304", "width": 640, "height": 480, "fps": 30},
    left: {"type": "intelrealsense", "serial_number_or_name": "335122271633", "width": 640, "height": 480, "fps": 30},
    right: {"type": "intelrealsense", "serial_number_or_name": "323622271837", "width": 640, "height": 480, "fps": 30}
  }' \
  --teleop.type=bi_yam_leader \
  --teleop.left_arm_port=5002 \
  --teleop.right_arm_port=5001 \
  --dataset.repo_id="HGLLL/bimanual-yam-demo" \
  --dataset.num_episodes=10 \
  --dataset.reset_time_s=25 \
  --dataset.single_task="Pick and place the object" \
  --display_data=true \
  --record_effort=true \
  --dataset.fps=30 \
  --dataset.vcodec=hevc_nvenc \
  --dataset.streaming_encoding=true \
  --dataset.encoder_threads=2
```

#### With Depth Video Recording (RealSense)

To additionally capture depth streams from RealSense cameras as lossless-quantized
video alongside RGB, use the custom `lerobot-record-with-depth` entry point. It
wraps `lerobot-record` and adds:

- A new `--depth_cams` flag listing cameras to also capture depth from. Every
  listed camera must be configured with `use_depth=true`.
- Automatic log-quantization of `uint16` depth (in mm) to 10 bits, packed into
  the Y plane of `yuv420p10le` and encoded with `libsvtav1` (crf=0). Precision
  is sub-mm near 0.1 m and ~2 mm near 3.0 m on the default log-scale.
- Per-feature `video.is_depth_map=True` is stamped on the dataset schema so the
  reader applies the inverse dequantization at load time.

> **Note:** depth is only supported from cameras whose backend exposes a
> `read_depth()` method. Currently that means Intel RealSense cameras.
> OpenCV / palm USB webcams will not produce a depth stream.

```bash
lerobot-record-with-depth \
  --robot.type=bi_yam_follower \
  --robot.left_arm_port=1235 \
  --robot.right_arm_port=1234 \
  --robot.cameras='{
    top:   {"type": "intelrealsense", "serial_number_or_name": "141722076304", "width": 640,  "height": 480, "fps": 30, "use_depth": true},
    left:  {"type": "intelrealsense", "serial_number_or_name": "335122271633", "width": 640, "height": 480, "fps": 30, "use_depth": true},
    right: {"type": "intelrealsense", "serial_number_or_name": "323622271837", "width": 640, "height": 480, "fps": 30, "use_depth": true}
  }' \
  --teleop.type=bi_yam_leader \
  --teleop.left_arm_port=5002 \
  --teleop.right_arm_port=5001 \
  --dataset.repo_id="${HF_USER}/bimanual-yam-depth-demo" \
  --dataset.num_episodes=10 \
  --dataset.reset_time_s=25 \
  --dataset.single_task="Pick and place the object" \
  --display_data=true \
  --depth_cams='[top, left, right]' \
  --min_depth_m=0.10 \
  --max_depth_m=1.00 \
  --dataset.fps=30 \
  --dataset.vcodec=hevc_nvenc \
  --dataset.streaming_encoding=true \
  --dataset.encoder_threads=2
```

Recorded features will include the usual `observation.images.{cam}` (RGB) **plus**
`observation.images.{cam}_depth` (uint16 mm → float32 m at read time) for every
camera listed in `--depth_cams`.

**Tuning depth range (close-range tasks)**

The depth range controls both the hardware clamp and the log-quantization
scale. Log-scale quantization puts more bits near the close range, so
**narrowing the range gives substantially tighter precision.**

| Flag            | Default | When to change                                                                        |
| --------------- | ------- | ------------------------------------------------------------------------------------- |
| `--min_depth_m` | `0.10`  | Raise if your closest object is always > some value (rare).                           |
| `--max_depth_m` | `3.00`  | **Lower for close-range manipulation.** For tabletop tasks use `1.00` or even `0.60`. |

For example, a tabletop pick-and-place task with objects at 0.2–0.8 m:

```bash
--min_depth_m=0.10 --max_depth_m=1.00
```

cuts the worst-case quantization error roughly 3× vs the default.

The chosen range is stored in each depth feature's info dict as
`video.depth_min_m` / `video.depth_max_m` / `video.depth_q_bits` so the
dataset is self-documenting. When loading the dataset later, align your
decoder to the stored range:

```python
from lerobot.datasets import LeRobotDataset
from lerobot.scripts.lerobot_record_with_depth import (
    encode_depth, decode_depth, set_depth_range,
)
ds = LeRobotDataset(..., depth_map_encoding_fn=encode_depth,
                         depth_map_decoding_fn=decode_depth)
info = ds.meta.features["observation.images.top_depth"]["info"]
set_depth_range(info["video.depth_min_m"], info["video.depth_max_m"])
# … now ds[i]["observation.images.top_depth"] decodes correctly in meters.
```

**Tuning codec** — edit `src/lerobot/scripts/lerobot_record_with_depth.py`:

- `DEPTH_CODEC` / `DEPTH_PIX_FMT` / `DEPTH_CRF` — codec knobs. Default
  `libsvtav1 / yuv420p10le / crf=0` is lossless given the 10-bit quantization.

#### With Tactile Sensor (XELA, optional)

To capture XELA tactile readings alongside the standard observation stream,
add `--robot.tactile_sensors='{...}'`. Each pad is keyed by a descriptive name
(e.g., `right_finger_r` for the right-jaw fingertip pad on the right gripper)
and surfaces in the dataset as `observation.tactile.<name>`.

> **Prerequisite:** `xela_server` must be running before `lerobot-record`
> launches. See the [Tactile Sensor (XELA, optional)](#tactile-sensor-xela-optional)
> section below for one-time setup, multi-sensor naming, calibrated forces,
> and how to stop the server after recording.

**Tactile only (no cameras)** — useful for sanity-checking the sensor pipeline
or training touch-only policies:

```bash
lerobot-record \
  --robot.type=bi_yam_follower \
  --robot.tactile_sensors='{right_finger_r: {type: xela, port: 5000, sensor_id: "1", model: XR1944}}' \
  --teleop.type=bi_yam_leader \
  --dataset.repo_id="${HF_USER}/bimanual-yam-tactile-demo" \
  --dataset.num_episodes=10 \
  --dataset.single_task="Pick and place with tactile feedback" \
  --display_data=true \
  --dataset.fps=30
```

**Tactile + RealSense cameras** — typical contact-rich manipulation recording:

```bash
lerobot-record \
  --robot.type=bi_yam_follower \
  --robot.left_arm_port=1235 \
  --robot.right_arm_port=1234 \
  --robot.cameras='{
    top:   {"type": "intelrealsense", "serial_number_or_name": "141722076304", "width": 640, "height": 480, "fps": 30},
    left:  {"type": "intelrealsense", "serial_number_or_name": "335122271633", "width": 640, "height": 480, "fps": 30},
    right: {"type": "intelrealsense", "serial_number_or_name": "323622271837", "width": 640, "height": 480, "fps": 30}
  }' \
  --robot.tactile_sensors='{right_finger_r: {type: xela, port: 5000, sensor_id: "1", model: XR1944}}' \
  --teleop.type=bi_yam_leader \
  --teleop.left_arm_port=5002 \
  --teleop.right_arm_port=5001 \
  --dataset.repo_id="${HF_USER}/bimanual-yam-tactile-vision-demo" \
  --dataset.num_episodes=10 \
  --dataset.reset_time_s=25 \
  --dataset.single_task="Pick and place with tactile + vision" \
  --display_data=true \
  --dataset.fps=30 \
  --dataset.vcodec=hevc_nvenc \
  --dataset.streaming_encoding=true \
  --dataset.encoder_threads=2
```

The recorded key is `observation.tactile.right_finger_r` with shape `(48,)`
(16 taxels × X/Y/Z) and dtype `float32`. To add more sensors, enable
XCAL-calibrated forces, or inspect a recorded episode, see the dedicated
[Tactile Sensor (XELA, optional)](#tactile-sensor-xela-optional) section.

#### Resuming an existing dataset

To append more episodes to a dataset that already exists on the Hub, pass
`--resume=true` together with `--dataset.root=<local writable path>`.
`LeRobotDataset.resume()` refuses to write into the default Hub snapshot
cache (under `~/.cache/huggingface/lerobot/...`) — that cache is
revision-safe and shared, so a recording session would corrupt it. The first
resume call materialises the dataset from the Hub into your `--dataset.root`,
and subsequent calls append directly to it.

> **`--dataset.num_episodes` semantics in resume mode:** the value is the
> count of episodes recorded **in this invocation** (the loop terminates
> once that many new episodes have been captured), *not* the new total.
> So if your dataset already has 2 episodes and you want to bring the total
> to 6, pass `--dataset.num_episodes=4`.

```bash
uv run lerobot-record \
  --robot.type=bi_yam_follower \
  --robot.tactile_sensors='{right_finger_r: {type: xela, port: 5000, sensor_id: "1", model: XR1944}}' \
  --teleop.type=bi_yam_leader \
  --dataset.repo_id="${HF_USER}/bimanual-yam-tactile-demo" \
  --dataset.root="$HOME/lerobot-data/bimanual-yam-tactile-demo" \
  --resume=true \
  --dataset.num_episodes=4 \
  --dataset.single_task="Pick and place with tactile feedback" \
  --display_data=false \
  --dataset.fps=30
```

If you'd rather not deal with a writable local root, simply record into a
fresh `--dataset.repo_id` (e.g. append `-v2`) and skip resume entirely.

> **Inspecting after resume — `--root` matters.** During and right after a
> resume session, your `--dataset.root` is the source of truth — it has
> the freshly-appended episodes. The default Hub snapshot cache
> (`~/.cache/huggingface/lerobot/<repo>`) only updates when you re-fetch
> from the Hub, so it can lag behind your local writes. Pass the same
> `--root` to inspect tools while iterating:
>
> ```bash
> uv run python examples/tactile/inspect_tactile_dataset.py \
>   --repo-id ${HF_USER}/bimanual-yam-tactile-demo \
>   --root "$HOME/lerobot-data/bimanual-yam-tactile-demo" \
>   --episode 2
> ```
>
> Once the session has pushed to the Hub and you want the default
> (no-`--root`) path to see the new episodes, refresh the cache:
>
> ```bash
> rm -rf ~/.cache/huggingface/lerobot/${HF_USER}/bimanual-yam-tactile-demo
> # Next inspect call without --root will re-fetch the latest Hub state.
> ```

### Configuration Parameters

#### Robot Configuration (`bi_yam_follower`)

- `robot.type`: Set to `bi_yam_follower`
- `robot.left_arm_port`: Server port for left follower arm (default: 1235)
- `robot.right_arm_port`: Server port for right follower arm (default: 1234)
- `robot.server_host`: Server hostname (default: "localhost")
- `robot.cameras`: Camera configurations (same as other robots)
- `robot.left_arm_max_relative_target`: Optional safety limit for left arm
- `robot.right_arm_max_relative_target`: Optional safety limit for right arm
- `robot.tactile_sensors`: Optional dict of tactile sensors keyed by name
  (e.g., `right_finger_r`). Each value picks a backend via `type:` (`xela`
  for hardware, `mock` for CI/offline development). For `xela`, fields are
  `host` (default `"auto"` — resolves to the LAN IP), `port` (default `5000`),
  `sensor_id` (default `"1"`), `model` (default `"XR1944"`),
  `use_calibrated` (default `false`), `tare_on_connect` (default `false`).
  See [Tactile Sensor (XELA, optional)](#tactile-sensor-xela-optional) for
  full details and the recorded-key naming convention.

#### Teleoperator Configuration (`bi_yam_leader`)

- `teleop.type`: Set to `bi_yam_leader`
- `teleop.left_arm_port`: Server port for left leader arm (default: 5002)
- `teleop.right_arm_port`: Server port for right leader arm (default: 5001)
- `teleop.server_host`: Server hostname (default: "localhost")

## Gripper Control with Teaching Handles

The teaching handles don't have physical grippers, but they have an **encoder button** (or digital input) that is used to command the follower gripper:

- **Press the encoder button**: Toggles the gripper between fully closed (0.0) and fully open (1.0)
- The leader handle input is read by the `bi_yam_leader` server and exposed as a binary gripper state
- The follower grippers mirror this open/close command in real-time
- Note: Currently only binary control is supported (no intermediate continuous positions)

## Architecture

### Data Flow

```
┌─────────────────┐         ┌─────────────────┐    ┌──────────────┐
│  Leader Arms    │         │  Follower Arms  │    │  XELA Pad    │
│  (Teaching      │         │  (Execution +   │    │  (right      │
│   Handles)      │         │   gripper)      │    │   finger r,  │
│                 │         │                 │    │   optional)  │
└────────┬────────┘         └────────▲────────┘    └──────┬───────┘
         │                           │                    │
         │ Read State                │ Send Actions       │ slCAN /tty
         │                           │                    │
    ┌────▼────┐              ┌───────┴─────┐         ┌────▼─────┐
    │ Leader  │              │  Follower   │         │  XELA    │
    │ Servers │              │  Servers    │         │  Server  │
    │ (5001,  │              │  (1234,     │         │  (WS     │
    │  5002)  │              │   1235)     │         │   :5000) │
    └────┬────┘              └───────▲─────┘         └────┬─────┘
         │                           │                    │
         │                           │                    │
    ┌────▼───────────────────────────┴────────────────────┴──────┐
    │              LeRobot Recording                              │
    │  - bi_yam_leader (teleoperator)                             │
    │  - bi_yam_follower (robot + tactile sensors)                │
    │  - Cameras                                                  │
    │  - Dataset writer                                           │
    └─────────────────────────────────────────────────────────────┘
```

### Server Process Details

The bimanual Yam setup uses server processes that wrap i2rt functionality:

**Unified Server (`run_bimanual_yam_server.py`)**:

- Starts all 4 arms (2 followers + 2 leaders) in a single process
- Easiest setup - just one command
- Recommended for most users

**Individual Server (`run_yam_server.py`)**:

- Runs a single arm server
- Useful for debugging or custom setups
- Requires running 4 separate instances

Each server:

1. Connects to a Yam arm via CAN
2. Provides gravity compensation
3. Exposes the robot state via a portal RPC server
4. Accepts position commands (for follower arms) or reads state (for leader arms)

**XELA Tactile Server (`run_xela_server.py`)**:

- Wraps the vendor `xela_server` binary in a managed Python subprocess.
- Reads slCAN frames from `/dev/ttyUSB0` (configured via the one-time
  `xela_conf` step) and serves the latest 4×4×3 reading over a local
  WebSocket on port 5000.
- Required only when recording with the XELA tactile pad; independent of
  the four arm servers.
- See [Tactile Sensor (XELA, optional)](#tactile-sensor-xela-optional) for
  setup and lifecycle commands.

## Troubleshooting

### CAN Interface Issues

If you get errors about missing CAN interfaces:

```bash
# Check if interfaces exist
ip link show | grep can

# Bring up an interface if it's down
sudo ip link set can_follower_r up
```

### Port Already in Use

If you get "address already in use" errors:

```bash
# Find and kill processes using the ports
lsof -ti:1234 | xargs kill -9
lsof -ti:1235 | xargs kill -9
lsof -ti:5001 | xargs kill -9
lsof -ti:5002 | xargs kill -9
```

### Connection Timeouts

If LeRobot can't connect to the servers:

1. Make sure all 4 i2rt server processes are running
2. Check that the servers started successfully without errors
3. Verify the port numbers match in both scripts

### Slow Control Loop

If you see warnings about slow control frequency:

- This usually means the system is overloaded
- Try reducing camera resolution or FPS
- Check CPU usage and close unnecessary applications
- Watch out for _first-second_ warmup warnings (camera cold start, NVENC
  encoder init): the FPS warning fires at the very first frame interval and
  may report a dramatic value (e.g., 4 Hz) even though the loop catches up to
  the target rate within a second. Confirm steady state before acting.

### Tactile Sensor Issues

If `lerobot-record --robot.tactile_sensors=...` errors out, work through:

**`Couldn't find a choice class for 'xela'`** — the backend wasn't registered
before draccus parsed the CLI. Make sure you're on the version that
pre-imports `XelaTactileConfig` in `src/lerobot/tactile/__init__.py`. If
installing from a fork, sync to the latest tactile commits.

**`WebSocket connection refused` / WS timeout** — `xela_server` isn't running,
or it's on a different host/port than what you passed. Check:

```bash
ps aux | grep xela_server          # is it running?
ss -ltnp | grep ':5000'            # is the port bound?
xela_conf -d socketcan -c slcan0   # re-init slCAN if it dropped
```

**XELA `host='auto'` resolves to the wrong IP** — the auto-resolver picks the
NIC that routes to a public IP. If you have multiple NICs (VPN, `docker0`,
or a separate management network), pass the LAN IP explicitly:

```bash
--robot.tactile_sensors='{right_finger_r: {type: xela, host: 192.168.1.86, port: 5000, sensor_id: "1", model: XR1944}}'
```

**Sensor disconnects mid-session** — the XELA stream is robust, but a USB
unplug or `slcand` reset will drop the WS. The recorder logs the close but
does not auto-reconnect mid-episode. Stop the run, fix the link, and resume.

**Frame validation crashes with shape `(48,)`** — symptom of the now-fixed
"1D tactile feature misclassified as a video stream" bug. Make sure you're
on the version that includes the `hw_to_dataset_features` rank split (commit
`a882a8da` or later).

**Stale orphan dataset directory** — if a previous tactile run crashed before
saving any episode, `~/.cache/huggingface/lerobot/<repo>` may contain only
`meta/info.json` and block re-creation with
`FileExistsError`. Delete the directory or rename it before re-recording.

**Slow record loop with tactile + cameras** — the WS read itself is cheap
(one small frame per loop), but on a 3-camera + Rerun setup the FPS warning
can fire transiently from camera/encoder warmup. See [Slow Control
Loop](#slow-control-loop) above; if steady-state rate is still below target,
toggle `--display_data=false` first.

## Advanced Usage

### Running Individual Server Processes

If you need more control or want to run servers separately, you can use the individual server script:

```bash
# Terminal 1: Right follower
python src/lerobot/robots/bi_yam_follower/run_yam_server.py \
  --can_channel can_follower_r \
  --gripper v3 \
  --mode follower \
  --server_port 1234

# Terminal 2: Left follower
python src/lerobot/robots/bi_yam_follower/run_yam_server.py \
  --can_channel can_follower_l \
  --gripper v3 \
  --mode follower \
  --server_port 1235

# Terminal 3: Right leader
python src/lerobot/robots/bi_yam_follower/run_yam_server.py \
  --can_channel can_leader_r \
  --gripper yam_teaching_handle \
  --mode leader \
  --server_port 5001

# Terminal 4: Left leader
python src/lerobot/robots/bi_yam_follower/run_yam_server.py \
  --can_channel can_leader_l \
  --gripper yam_teaching_handle \
  --mode leader \
  --server_port 5002
```

### Visualizer Mode

You can visualize arm movements using MuJoCo:

```bash
# Local visualization (directly connected to hardware)
python src/lerobot/robots/bi_yam_follower/run_yam_server.py \
  --can_channel can_follower_r \
  --gripper v3 \
  --mode visualizer_local

# Remote visualization (connect to running server)
python src/lerobot/robots/bi_yam_follower/run_yam_server.py \
  --server_host localhost \
  --server_port 1234 \
  --mode visualizer_remote
```

### Without Teleoperation

You can also use the bimanual Yam follower arms with a trained policy (without teleop):

```bash
lerobot-record \
  --robot.type=bi_yam_follower \
  --robot.left_arm_port=1235 \
  --robot.right_arm_port=1234 \
  --robot.cameras='{...}' \
  --policy.path=path/to/trained/policy \
  --dataset.repo_id=${HF_USER}/bimanual-yam-eval \
  --dataset.num_episodes=5
```

## Tactile Sensor (XELA, optional)

A single XELA XR1944 tactile pad mounted on the right-side fingertip of the right arm's
parallel gripper, served by [`xela_server`](../../tactile/xela/README.md) v1.7.x over
a local WebSocket.

### Per-boot setup

> **First time on this machine?** Complete the
> [First-time setup](../../tactile/xela/README.md#first-time-setup-one-time-per-machine)
> in the XELA backend README first (apt `can-utils`, `/etc/xela` directory,
> unpack the vendor `appimage.zip`, `PATH`, interactive `xela_conf`). The
> commands below assume those one-time steps are already done.

Each boot, bring slCAN up and (re)write `xServ.ini` if it doesn't already
match your hardware:

```bash
sudo slcand -o -s8 -t hw -S 3000000 /dev/ttyUSB0 slcan0
sudo ifconfig slcan0 up
xela_conf -d socketcan -c slcan0   # writes /etc/xela/xServ.ini if missing — answer y<Enter> to save
```

### Per session

```bash
# Terminal A — start XELA server (leave running)
python src/lerobot/robots/bi_yam_follower/run_xela_server.py

# Terminal B — start arm servers (existing pattern, leave running)
python src/lerobot/robots/bi_yam_follower/run_bimanual_yam_server.py

# Terminal C — record with tactile included.
# `host` is omitted; the client's default "auto" resolves to the LAN IP
# xela_server binds to (see the XELA backend README's "Bring-up checklist").
lerobot-record \
  --robot.type=bi_yam_follower \
  --robot.tactile_sensors='{right_finger_r: {type: xela, port: 5000, sensor_id: "1", model: XR1944}}' \
  --teleop.type=bi_yam_leader \
  --dataset.repo_id="${HF_USER}/bimanual-yam-tactile-demo" \
  --dataset.num_episodes=10 \
  --dataset.single_task="Pick and place with tactile feedback" \
  --display_data=true \
  --dataset.fps=30
```

For tactile combined with cameras, see [Step 3 → With Tactile Sensor (XELA, optional)](#with-tactile-sensor-xela-optional)
above. To append more episodes to an already-published tactile dataset, see
[Step 3 → Resuming an existing dataset](#resuming-an-existing-dataset).

### Stopping `xela_server` after the session

See [Terminating `xela_server`](../../tactile/xela/README.md#terminating-xela_server)
in the XELA backend README — covers `kill $!` for foreground-launched processes,
`pkill -f xela_server` for stale instances, and the SIGKILL / `Ctrl+Shift+\`
fallbacks if SIGTERM hangs.

### Recorded keys

| Key                                  | Shape   | Dtype     | Notes                                                                              |
| ------------------------------------ | ------- | --------- | ---------------------------------------------------------------------------------- |
| `observation.tactile.right_finger_r` | `(48,)` | `float32` | Raw uint16 magnetic-field readings cast losslessly to float32 (16 taxels × X/Y/Z). |

**Calibrated forces (XCAL):** set `"use_calibrated": true` on the sensor
config to additionally record `observation.tactile.right_finger_r.cal` —
a parallel `(48,)` float32 column populated from each frame's `calibrated`
field (XCAL-calibrated forces in Newtons). If the vendor's `.xcal` files
are not installed at the `xela_server` side, the column is filled with
zeros and a one-shot `ERROR` is logged on the first null-calibrated frame
so the gap is visible in the logs without breaking recording. Raw uint16
readings are always recorded regardless of this flag.

### Naming convention for adding more sensors

`observation.tactile.<arm>_finger_<side>` where `arm ∈ {left, right}` and
`side ∈ {l, r}` (which jaw of the parallel gripper). Add additional entries to
`--robot.tactile_sensors='{...}'` and the keys appear in the dataset automatically.

### Inspecting tactile data after recording

Use [`examples/tactile/inspect_tactile_dataset.py`](../../../../examples/tactile/inspect_tactile_dataset.py)
to verify schema and visualise the recorded data:

```bash
# Headless sanity check — prints schema + per-axis (X, Y, Z) min/max/mean/std
uv run python examples/tactile/inspect_tactile_dataset.py \
  --repo-id ${HF_USER}/bimanual-yam-tactile-demo --episode 0

# Animated 4×4 heatmap of contact magnitude across the episode
uv run python examples/tactile/inspect_tactile_dataset.py \
  --repo-id ${HF_USER}/bimanual-yam-tactile-demo --episode 0 --mode heatmap

# Run all views in sequence (summary, time-series, heatmap, single-frame)
uv run python examples/tactile/inspect_tactile_dataset.py \
  --repo-id ${HF_USER}/bimanual-yam-tactile-demo --episode 0 --mode all
```

## References

- **i2rt library**: Python library for controlling Yam arm hardware (install via `pip install -e '.[yam]'`)
  - GitHub: [i2rt-robotics/i2rt](https://github.com/i2rt-robotics/i2rt)
  - Used internally by `run_yam_server.py` for hardware communication
- **portal**: RPC framework for client-server communication (installed with yam dependencies)
- **LeRobot documentation**: See main docs for training and evaluation workflows
- **XELA backend (LeRobot)**: [`src/lerobot/tactile/xela/README.md`](../../tactile/xela/README.md)
  — WebSocket protocol details, sensor model table (`XR1944`, `XR1946`,
  `XR2244`, `XR1922`), and calibration notes.
- **Inspect tactile data**: [`examples/tactile/inspect_tactile_dataset.py`](../../../../examples/tactile/inspect_tactile_dataset.py)
  — schema check, time-series plot, 4×4 contact heatmap, and per-axis
  spatial frame views for any recorded episode.
- **XELA Software Manual**: vendor documentation for `xela_server`,
  `xela_conf`, slCAN setup, and per-model taxel layouts (consult the manual
  shipped with your sensor).
