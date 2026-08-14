#!/usr/bin/env python3
"""Capture serial output from the board for a fixed duration.

Usage: capture.py [--reset] [seconds] [port]

This board's Type-C port is native USB-Serial/JTAG. Opening it with the
wrong DTR/RTS order reboots the chip and re-inits the AMOLED. Default
matches `idf.py monitor --no-reset` so the face stays on; pass --reset
only when you need the boot banner.
"""
from __future__ import annotations

import argparse
import sys
import time

import serial

DEFAULT_PORT = "/dev/cu.usbmodem1101"

# Same polarity as esp-idf-monitor: LOW=asserted (True), HIGH=idle (False).
_LOW = True
_HIGH = False


def open_serial(port: str, *, reset: bool) -> serial.Serial:
    # Mirror idf_monitor SerialReader.open_serial: configure DTR/RTS before
    # open, then idle RTS first after open so EN does not glitch.
    ser = serial.serial_for_url(port, 115200, do_not_open=True)
    ser.timeout = 0.2
    ser.rts = _LOW
    ser.dtr = _LOW
    ser.open()
    ser.rts = _HIGH
    ser.dtr = _HIGH
    if reset:
        ser.rts = _LOW
        time.sleep(0.1)
        ser.rts = _HIGH
        ser.reset_input_buffer()
    return ser


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("seconds", nargs="?", type=float, default=8.0)
    ap.add_argument("port", nargs="?", default=DEFAULT_PORT)
    ap.add_argument(
        "--reset",
        action="store_true",
        help="Pulse DTR/RTS (reboots the chip and re-inits the panel)",
    )
    args = ap.parse_args()

    with open_serial(args.port, reset=args.reset) as ser:
        deadline = time.time() + args.seconds
        while time.time() < deadline:
            chunk = ser.read(4096)
            if chunk:
                sys.stdout.write(chunk.decode("utf-8", errors="replace"))
                sys.stdout.flush()
    return 0


if __name__ == "__main__":
    sys.exit(main())
