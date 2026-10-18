#!/usr/bin/env python3
"""Watch for an ESP32 on USB serial and print its MAC address.

The script polls serial-device paths and asks esptool to read the MAC when a
new device appears. It deliberately does not flash, erase, or otherwise alter
the ESP32.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path


MAC_RE = re.compile(r"(?i)\b(?:[0-9a-f]{2}[:-]){5}[0-9a-f]{2}\b")


def serial_devices() -> list[str]:
    """Return likely USB serial devices, in stable order."""
    candidates: set[str] = set()

    if sys.platform == "darwin":
        # Ignore Bluetooth/debug-console devices; probing those makes an
        # unplugged ESP32 look like a stalled monitor.
        for pattern in ("cu.usb*", "cu.SLAB*", "cu.wch*"):
            candidates.update(str(path) for path in Path("/dev").glob(pattern))
    elif os.name == "nt":
        # Windows COM ports are handled by an explicit --port, since there is
        # no reliable stdlib-only way to enumerate them.
        return []
    else:
        for pattern in ("/dev/ttyUSB*", "/dev/ttyACM*"):
            candidates.update(str(path) for path in Path("/dev").glob(pattern[5:]))

    return sorted(path for path in candidates if os.path.exists(path))


def esptool_command() -> list[str]:
    """Prefer an esptool executable, then the Python module."""
    local_venv = Path(__file__).with_name(".venv") / "bin" / "esptool"
    if local_venv.is_file():
        return [str(local_venv)]
    executable = shutil.which("esptool") or shutil.which("esptool.py")
    if executable:
        return [executable]
    return [sys.executable, "-m", "esptool"]


def read_mac(port: str, timeout: float) -> str | None:
    """Read and normalize the MAC returned by esptool."""
    base = esptool_command()
    commands = [base + ["--port", port, "read-mac"], base + ["--port", port, "read_mac"]]

    for command in commands:
        try:
            result = subprocess.run(
                command,
                capture_output=True,
                text=True,
                timeout=timeout,
                check=False,
            )
        except (FileNotFoundError, subprocess.TimeoutExpired):
            continue

        output = f"{result.stdout}\n{result.stderr}"
        matches = MAC_RE.findall(output)
        if result.returncode == 0 and matches:
            return matches[-1].replace("-", ":").upper()

    return None


def print_result(port: str, mac: str, as_json: bool) -> None:
    if as_json:
        print(json.dumps({"port": port, "mac": mac}), flush=True)
    else:
        # This single-line form is intentionally easy to scan or send to a
        # label printer workflow.
        print(f"ESP32 MAC: {mac}    PORT: {port}", flush=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Monitor USB serial ports and print the MAC of each ESP32 found."
    )
    parser.add_argument("--port", help="Monitor one port (for example /dev/cu.SLAB_USBtoUART).")
    parser.add_argument("--once", action="store_true", help="Exit after the first MAC is read.")
    parser.add_argument("--interval", type=float, default=1.0, help="Polling interval in seconds (default: 1).")
    parser.add_argument("--timeout", type=float, default=8.0, help="esptool timeout in seconds (default: 8).")
    parser.add_argument("--json", action="store_true", help="Print one JSON object per detected board.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.interval <= 0 or args.timeout <= 0:
        print("--interval and --timeout must be greater than zero", file=sys.stderr)
        return 2

    seen_ports: set[str] = set()
    if not args.json:
        print("Waiting for ESP32... Press Ctrl-C to stop.", flush=True)

    try:
        while True:
            current_ports = [args.port] if args.port else serial_devices()
            for port in current_ports:
                if port in seen_ports:
                    continue
                mac = read_mac(port, args.timeout)
                if mac:
                    print_result(port, mac, args.json)
                    seen_ports.add(port)
                    if args.once:
                        return 0
                elif args.port:
                    # An explicit port may not be connected yet; retry it.
                    continue

            # A removed port is eligible for detection again when reconnected.
            seen_ports.intersection_update(current_ports)
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\nStopped.", file=sys.stderr)
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
