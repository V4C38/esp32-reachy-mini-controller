#!/usr/bin/env python3
"""Low-rate /api/status soak. Serial must stay closed.

Polls once per second so the measurement path cannot stall the ESP32
USB-Serial/JTAG stack. Prints a one-line summary and exits non-zero if the
controller socket flips during the window.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.error
import urllib.request


def fetch(url: str) -> dict:
    with urllib.request.urlopen(url, timeout=2.0) as resp:
        return json.load(resp)


def main() -> int:
    parser = argparse.ArgumentParser(description="Soak Motion Controller /api/status")
    parser.add_argument("--url", default="http://127.0.0.1:8766/api/status")
    parser.add_argument("--seconds", type=float, default=90.0)
    parser.add_argument(
        "--require-connected",
        action="store_true",
        help="Fail if the controller is not connected at the first sample",
    )
    args = parser.parse_args()

    t0 = time.monotonic()
    samples = 0
    connected = 0
    flips = 0
    prev = None
    last = None
    errors = 0
    print(f"soak {args.url} for {args.seconds:.0f}s (1 Hz, serial closed)", flush=True)
    while time.monotonic() - t0 < args.seconds:
        try:
            last = fetch(args.url)
            samples += 1
            now = bool(last.get("connected"))
            if now:
                connected += 1
            if prev is not None and now != prev:
                flips += 1
                print(
                    f"FLIP t={time.monotonic() - t0:.1f}s connected={now} "
                    f"diag={last.get('last_diag')} peer={last.get('peer')} "
                    f"boot={last.get('boot_id')} absents={last.get('absents')}",
                    flush=True,
                )
            prev = now
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError) as exc:
            errors += 1
            print(f"error t={time.monotonic() - t0:.1f}s {exc}", flush=True)
        remaining = args.seconds - (time.monotonic() - t0)
        time.sleep(min(1.0, max(0.0, remaining)))

    print(
        json.dumps(
            {
                "samples": samples,
                "connected": connected,
                "flips": flips,
                "errors": errors,
                "last": last,
            },
            sort_keys=True,
        )
    )
    if args.require_connected and (samples == 0 or connected == 0):
        return 2
    if flips or errors:
        return 1
    if args.require_connected and connected != samples:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
