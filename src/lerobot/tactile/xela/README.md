# XELA Tactile Backend

A WebSocket client for [XELA Robotics](https://xelarobotics.com/) tactile sensors served
by `xela_server` (v1.7.x). Decodes the JSON wire format documented in §"Data format" of
the XELA Software Manual into a flat `(N,)` `np.float32` vector per read.

## Supported models

| Model    | Layout                       | Channels |
| -------- | ---------------------------- | -------- |
| `XR1944` | 4×4 taxels × 3 axes          | 48       |
| `XR1946` | 4×6 taxels × 3 axes          | 72       |
| `XR2244` | 4×4 (magnetic-comp) × 3 axes | 48       |
| `XR1922` | 4×4 × 3 axes                 | 48       |

Add new models in `configuration_xela.py:TAXELS_BY_MODEL`.

## Wire protocol

Per the manual (p. 37), each WebSocket message is a JSON object:

```json
{
  "message": 786,
  "time": 1619157045.6195483,
  "sensors": 1,
  "1": {
    "data": "FA00,FB00,FC00,...",
    "sensor": "1",
    "taxels": 16,
    "model": "XR1944",
    "calibrated": null
  }
}
```

`data` is a comma-separated string of 4-character hex codes (uint16), 3 per taxel
(X, Y, Z). The first message after connect is `{"message": "Welcome", ...}` and is
skipped. Sequence numbers (`message`) must be monotonic; out-of-order frames are dropped.

## First-time setup (one-time per machine)

Skip this section if `xela_conf` and `xela_server` are already on your `PATH`
and `/etc/xela/xServ.ini` already describes your sensor (run `which xela_conf`
to check).

### Hardware

XELA ships a **VScom USB-CAN Plus** adapter with two USB cables — **both must
be connected** to the host. The second cable is sensor-side power, not data,
and the converter will fail silently if it's missing (vendor manual §"Common
errors", row "Message not sent: Transmit operation timed out").

### Find your VScom adapter's `/dev/ttyUSB*` path

The slCAN bring-up command (`sudo slcand … /dev/ttyUSB0 slcan0`) hard-codes
`/dev/ttyUSB0`, but Linux assigns USB-serial numbers in the order devices
are plugged in. If you have any other USB-serial device on the host (a
microcontroller, GPS, or another arm's CAN-USB adapter) the VScom may end
up as `/dev/ttyUSB1`, `/dev/ttyUSB2`, etc.

**Diff-based discovery** (vendor-recommended, no special tools):

```bash
# (1) Before plugging in the VScom adapter, list current ttyUSB devices.
ls /dev/ttyUSB*
# (Output may be "ls: cannot access '/dev/ttyUSB*': No such file or directory"
#  if you have nothing else plugged in — that's fine.)

# (2) Plug in the VScom USB-CAN Plus. Confirm BOTH USB cables are connected.

# (3) Run the listing again. The newly-appeared entry is your adapter.
ls /dev/ttyUSB*
```

**Or watch the kernel log** as the adapter is hotplugged (works even if you
forgot to take the "before" snapshot):

```bash
# Plug the adapter, then immediately:
dmesg | grep -E 'tty(USB|ACM)' | tail -5
# Expected last line is something like:
# usb 1-X.Y: ch341-uart converter now attached to ttyUSB0
```

If your adapter ends up at a path other than `/dev/ttyUSB0`, substitute it
into every `slcand …` and `xela_conf …` command below.

> **Pinning the path across reboots (optional).** A `udev` rule keyed on the
> adapter's USB serial / vendor:product IDs can map it to a stable symlink
> like `/dev/xela-ttyUSB`. This is out of scope here, but `lsusb -v` shows
> the IDs you'd need; the [Arch Linux wiki on udev](https://wiki.archlinux.org/title/Udev)
> has a recipe.

### Install `can-utils`

`slcand` (used to bring the slCAN interface up at boot) ships in `can-utils`,
which isn't a default Ubuntu package:

```bash
sudo apt update && sudo apt install can-utils
```

### Install the XELA software suite

The vendor distributes the four binaries (`xela_conf`, `xela_server`,
`xela_viz`, `xela_log`) as `appimage.zip`. The canonical install location is
`/etc/xela`, which the binaries also use to find/write `xServ.ini`:

```bash
# (1) Create the directory with world-writable perms — the binaries write
#     xServ.ini here at runtime (vendor manual §"Common errors": IOError
#     [Errno 2] if the dir is missing, [Errno 13] if perms are wrong).
sudo mkdir -p /etc/xela
sudo chmod -R 777 /etc/xela

# (2) Unpack the vendor zip into /etc/xela
unzip ~/Downloads/appimage.zip -d /etc/xela

# (3) Put /etc/xela on PATH so `xela_conf`, `xela_server`, etc. resolve
echo 'export PATH=$PATH:/etc/xela' >> ~/.bashrc
source ~/.bashrc
```

Verify with `which xela_server` and a tab-complete check (`xela_<TAB><TAB>`
should list all four binaries).

### Generate `xServ.ini`

`xela_conf` is **interactive** — it scans the CAN bus, prints the sensors it
finds, and prompts `Save: y/n`. Answer `y` and press Enter to write
`/etc/xela/xServ.ini`:

```bash
# Bring slCAN up first so xela_conf has a bus to scan (see Bring-up checklist below)
xela_conf -d socketcan -c slcan0
# Output ends with: "...is correct, please enter y and press Enter to save..."
# Type: y<Enter>
```

> The default `xServ.ini` shipped in `appimage.zip` is in **simulation mode**
> (`bustype = sim`, `model = uSPa44`) and will **not** read your hardware
> until `xela_conf -d socketcan -c slcan0` rewrites it with the detected
> model.

## Bring-up checklist (per boot)

```bash
# (1) Activate slcan (per boot — wire to systemd in production)
sudo slcand -o -s8 -t hw -S 3000000 /dev/ttyUSB0 slcan0
sudo ifconfig slcan0 up

# (2) Generate xServ.ini if you haven't yet (see "First-time setup" above)
xela_conf -d socketcan -c slcan0    # one-shot, writes /etc/xela/xServ.ini

# (3) Start the server (leave running, e.g. in a dedicated terminal)
python src/lerobot/robots/bi_yam_follower/run_xela_server.py
# or directly: /etc/xela/xela_server -f /etc/xela/xServ.ini -p 5000 --noros &
# (NOTE: --ip is intentionally omitted — xela_server v1.7.6 build 158509 silently
#  ignores it and always binds to the host's primary NIC IP, e.g. 192.168.x.x.
#  Our client auto-resolves to the same IP at connect time, so no flag is needed.)

# (4) Smoke-test the stream — `--host` defaults to "auto" (resolves to LAN IP)
python -m lerobot.tactile.xela.xela_tactile --port 5000 --sensor-id 1
```

### What a healthy server looks like

After step (3), `xela_server` prints a status line that updates in place:

```
'ip': '192.168.1.34', 'port': 5000, 'connections': 0, 'sensors': ['XR1944'], ...
... Users: 0 - CAN: 1182.72/s - Server: 95.2Hz
```

- `Server: ~95–100 Hz` and a non-zero `CAN: …/s` rate confirm the slCAN side
  is alive.
- `Users: 0` is the WebSocket client count; it ticks up to `1` the moment
  `lerobot-record` (or any other client) connects. Watching this counter is
  the simplest way to confirm a recording session has actually attached to
  the server.

## Terminating `xela_server`

`Ctrl+C` does **not** stop a backgrounded (`&`) process — it only signals the
foreground process group. Use one of:

| Command                | When to use                                                                                        |
| ---------------------- | -------------------------------------------------------------------------------------------------- |
| `kill $!`              | Cleanest, when you started it via `&` in the same shell. `$!` is the most-recent backgrounded PID. |
| `pkill -f xela_server` | When `$!` is no longer in scope or there are stale instances. SIGTERM by default — graceful.       |
| `pkill -9 xela_server` | Manual's fallback if SIGTERM hangs (XELA Software Manual v1.7.6, p. 39).                           |
| `fg` then `Ctrl+C`     | Bring the bg job back to fg, then SIGINT it.                                                       |

## Optional vendor tools

`appimage.zip` ships two extra binaries that are useful during sensor bring-up
but are **not** required for `lerobot-record`:

### `xela_viz` — live GUI visualization

Pops a Qt/SDL2 window showing the 4×4 (or 4×6) taxel grid with circles whose
size and colour track contact pressure in real time. Useful for confirming the
sensor is wired correctly and responds to touch _before_ you depend on it
inside a LeRobot recording.

```bash
# In a separate terminal — xela_server must already be running.
xela_viz
```

The `[viz]` block in `xServ.ini` (`arrows`, `grid`, `transparency`,
`origins`, `fps`, `ups`, `temp_show`) toggles overlay options. Vendor manual
§"XELA Visualisation" documents each.

### `xela_log` — offline data logger

Captures sensor frames to disk without going through a LeRobot dataset.
Useful for very fast (>30 Hz) raw captures or for sharing reproducer logs
when filing vendor bug reports (vendor checklist: "attach files from
`/etc/xela/LOG/`"). See the vendor manual §"How to use XELA Logger" for
arguments.

## After recording — inspecting the data

See [`examples/tactile/inspect_tactile_dataset.py`](../../../../examples/tactile/inspect_tactile_dataset.py)
for ready-to-run modes (`summary`, `timeseries`, `heatmap`, `frame`, `all`).

## Failure modes

### LeRobot client side

| Condition                                    | Behaviour                                                                                                                     |
| -------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------- |
| `xela_server` not running at connect         | `XelaTactileSensor.async_read()` raises `TimeoutError` after `receive_timeout_s`.                                             |
| WS disconnect mid-episode                    | Reader auto-reconnects (capped exponential backoff to 2 s); `async_read` returns last-good-frame; one warning per disconnect. |
| Sequence regression                          | Frame dropped silently.                                                                                                       |
| Stale frame (`now − latest_timestamp > 1 s`) | Warning logged each call, data still returned.                                                                                |
| Model mismatch (xServ.ini vs config)         | Frame dropped, warning logged.                                                                                                |

### Vendor binary side (xela_conf / xela_server)

Lifted from XELA Software Manual v1.7.6 §"Common errors" (p. 39–40).

| Vendor error                                                                                     | Cause / fix                                                                                                                                                                                                |
| ------------------------------------------------------------------------------------------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `xela_conf`: `Message not sent: Transmit operation timed out`                                    | The CAN-USB converter's USB power cable isn't connected, or the DSUB-9 link to the converter is loose. Both VScom USB cables must be plugged in.                                                           |
| `Could not start CAN: OSError: [Errno 19] No such device`                                        | The slCAN network interface isn't up. Re-run the per-boot bring-up: `sudo slcand …; sudo ifconfig slcan0 up`.                                                                                              |
| `Error connecting to CAN: IOError: [Errno 19] No such device`                                    | No CAN device found at all — adapter unplugged, driver missing, or `bustype/channel` in `xServ.ini` doesn't match the hardware.                                                                            |
| `Error writing config file: IOError: [Errno 2] No such file or directory: '/etc/xela/xServ.ini'` | `/etc/xela` doesn't exist or isn't writable. Re-run the [First-time setup](#first-time-setup-one-time-per-machine) `mkdir`/`chmod 777` steps.                                                              |
| `xela_server` runs but `xela_viz` shows no data                                                  | On Windows: AV software blocks the local WS connection — disable AV. On Linux this combination is rare; check the `xServ.ini` `[sensor]` block for a model mismatch first.                                 |
| New CAN-USB converter not recognised                                                             | Check that the converter is supported (`socketcan`/`esd`/`pcan`), `xServ.ini` has the right `bustype` + `channel`, the network is up, and the `ctrl_id` matches the hardware (vendor manual §"CAN types"). |
| Program won't respond to `Ctrl+C`                                                                | Use `Ctrl+Shift+\` (SIGQUIT) or `pkill -9 xela_server` — see [Terminating `xela_server`](#terminating-xela_server) above.                                                                                  |
