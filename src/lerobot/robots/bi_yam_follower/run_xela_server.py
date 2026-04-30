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

"""Operator wrapper for launching XELA Server v1.7.x with sensible defaults.

Run this in its own terminal *after* the slcan bus is up:

    sudo slcand -o -s8 -t hw -S 3000000 /dev/ttyUSB0 slcan0
    sudo ifconfig slcan0 up
    python src/lerobot/robots/bi_yam_follower/run_xela_server.py
"""

import argparse
import signal
import subprocess
import sys
from pathlib import Path

DEFAULT_BIN = Path("/etc/xela/xela_server")
DEFAULT_INI = Path("/etc/xela/xServ.ini")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--xela-bin", type=Path, default=DEFAULT_BIN,
                    help="Path to the xela_server AppImage (default: /etc/xela/xela_server).")
    ap.add_argument("--config", type=Path, default=DEFAULT_INI,
                    help="Path to xServ.ini (default: /etc/xela/xServ.ini).")
    ap.add_argument("--ip", default="127.0.0.1",
                    help="Server bind IP. Default 127.0.0.1 to keep the WS local-only.")
    ap.add_argument("--port", type=int, default=5000,
                    help="Server WebSocket port. Default 5000 matches XELA's manual.")
    ap.add_argument("--noros", action="store_true", default=True,
                    help="Pass --noros (default true). Use --noros=false to enable ROS bridge.")
    args = ap.parse_args()

    if not args.xela_bin.exists():
        print(f"error: xela_server binary not found at {args.xela_bin}", file=sys.stderr)
        return 1
    if not args.config.exists():
        print(f"error: xServ.ini not found at {args.config}. "
              f"Run `xela_conf -d socketcan -c slcan0` first.", file=sys.stderr)
        return 1

    # XELA's AppImage CLI requires LONG forms (--ip, --port) — short -i / -p are
    # silently ignored and the server falls back to its default (first NIC IP).
    cmd = [str(args.xela_bin), "-f", str(args.config),
           "--ip", args.ip, "-p", str(args.port)]
    if args.noros:
        cmd.append("--noros")

    print(f"$ {' '.join(cmd)}", flush=True)
    proc = subprocess.Popen(cmd)

    def _forward_sigint(signum, frame):
        proc.send_signal(signum)

    signal.signal(signal.SIGINT, _forward_sigint)
    signal.signal(signal.SIGTERM, _forward_sigint)

    return proc.wait()


if __name__ == "__main__":
    sys.exit(main())
