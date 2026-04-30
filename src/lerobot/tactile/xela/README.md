# XELA Tactile Backend

A WebSocket client for [XELA Robotics](https://xelarobotics.com/) tactile sensors served
by `xela_server` (v1.7.x). Decodes the JSON wire format documented in §"Data format" of
the XELA Software Manual into a flat `(N,)` `np.float32` vector per read.

## Supported models

| Model | Layout | Channels |
| --- | --- | --- |
| `XR1944` | 4×4 taxels × 3 axes | 48 |
| `XR1946` | 4×6 taxels × 3 axes | 72 |
| `XR2244` | 4×4 (magnetic-comp) × 3 axes | 48 |
| `XR1922` | 4×4 × 3 axes | 48 |

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

## Bring-up checklist

```bash
# (1) Activate slcan (one-time per boot — wire to systemd in production)
sudo slcand -o -s8 -t hw -S 3000000 /dev/ttyUSB0 slcan0
sudo ifconfig slcan0 up

# (2) Generate xServ.ini if you haven't yet
xela_conf -d socketcan -c slcan0    # one-shot, writes /etc/xela/xServ.ini

# (3) Start the server (leave running)
python src/lerobot/robots/bi_yam_follower/run_xela_server.py
# or directly: /etc/xela/xela_server -f /etc/xela/xServ.ini -i 127.0.0.1 -p 5000 --noros

# (4) Smoke-test the stream from Python
python -m lerobot.tactile.xela.xela_tactile --host 127.0.0.1 --port 5000 --sensor-id 1
```

## Failure modes

| Condition | Behaviour |
| --- | --- |
| `xela_server` not running at connect | `XelaTactileSensor.async_read()` raises `TimeoutError` after `receive_timeout_s`. |
| WS disconnect mid-episode | Reader auto-reconnects (capped exponential backoff to 2 s); `async_read` returns last-good-frame; one warning per disconnect. |
| Sequence regression | Frame dropped silently. |
| Stale frame (`now − latest_timestamp > 1 s`) | Warning logged each call, data still returned. |
| Model mismatch (xServ.ini vs config) | Frame dropped, warning logged. |
