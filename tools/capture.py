#!/usr/bin/env python3
"""Capture serial output from the board for a fixed duration.

Usage: capture.py [seconds] [port]

Unlike `idf.py monitor` this is non-interactive, so it can be scripted.
"""
import sys
import time

import serial

DEFAULT_PORT = "/dev/cu.usbmodem1101"


def main() -> int:
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0
    port = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_PORT

    with serial.Serial(port, 115200, timeout=0.2) as ser:
        # Pulse DTR/RTS the way esptool does to reset the chip, so we always
        # capture the boot banner rather than joining mid-stream.
        ser.setDTR(False)
        ser.setRTS(True)
        time.sleep(0.1)
        ser.setRTS(False)
        ser.reset_input_buffer()

        deadline = time.time() + duration
        while time.time() < deadline:
            chunk = ser.read(4096)
            if chunk:
                sys.stdout.write(chunk.decode("utf-8", errors="replace"))
                sys.stdout.flush()
    return 0


if __name__ == "__main__":
    sys.exit(main())
