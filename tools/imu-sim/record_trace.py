#!/usr/bin/env python3
"""Capture IMU CSV traces from device serial (CONFIG_RMC_TRACE lines)."""
from __future__ import annotations
import argparse, sys, time
import serial

def main():
    p = argparse.ArgumentParser()
    p.add_argument("out_csv")
    p.add_argument("--port", default="/dev/cu.usbmodem101")
    p.add_argument("--seconds", type=float, default=10.0)
    args = p.parse_args()
    with serial.Serial(args.port, 115200, timeout=0.2) as ser, open(args.out_csv, "w") as f:
        f.write("t_ms,qw,qx,qy,qz,px,py,pz,ax,ay,az,gx,gy,gz,still,engaged\n")
        deadline = time.time() + args.seconds
        while time.time() < deadline:
            line = ser.readline().decode("utf-8", errors="replace").strip()
            if line.startswith("RMC_TRACE,"):
                f.write(line[len("RMC_TRACE,"):] + "\n")
    print("wrote", args.out_csv)

if __name__ == "__main__":
    main()
