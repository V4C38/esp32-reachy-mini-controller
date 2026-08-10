#!/usr/bin/env python3
"""Play scripted controller_state sequences against the Motion Controller app."""

from __future__ import annotations

import argparse
import asyncio
import json
import sys
import time

try:
    import websockets
except ImportError:
    print("pip install websockets", file=sys.stderr)
    sys.exit(1)

from scipy.spatial.transform import Rotation as R

DEFAULT_URL = "ws://127.0.0.1:8766/ws"


def wxyz(roll: float, pitch: float, yaw: float) -> list[float]:
    q = R.from_euler("xyz", [roll, pitch, yaw], degrees=False).as_quat()
    return [float(q[3]), float(q[0]), float(q[1]), float(q[2])]


async def send_state(ws, seq, q, p, engaged, gain=1.0, ready=True):
    await ws.send(json.dumps({
        "type": "controller_state", "seq": seq, "q": q, "p": list(p),
        "engaged": engaged, "gain": gain, "ready": ready,
    }))


async def stream(ws, duration, q_fn, p_fn, engaged, gain=1.0, hz=20.0):
    dt = 1.0 / hz
    t0 = time.monotonic()
    seq = 0
    while time.monotonic() - t0 < duration:
        t = time.monotonic() - t0
        await send_state(ws, seq, q_fn(t), p_fn(t), engaged, gain=gain)
        seq += 1
        await asyncio.sleep(dt)


SCENARIOS = {}

def scenario(name):
    def deco(fn):
        SCENARIOS[name] = fn
        return fn
    return deco


@scenario("engage_rotate_release")
async def engage_rotate_release(ws):
    await stream(ws, 0.5, lambda t: wxyz(0, 0, 0), lambda t: (0, 0, 0), True)
    await stream(ws, 1.0, lambda t: wxyz(0, 0, 0.35 * min(1.0, t)), lambda t: (0, 0, 0), True)
    await stream(ws, 0.5, lambda t: wxyz(0, 0, 0.35), lambda t: (0, 0, 0), False)
    print("engage_rotate_release done")


@scenario("translate_hold")
async def translate_hold(ws):
    await stream(ws, 0.3, lambda t: wxyz(0, 0, 0), lambda t: (0, 0, 0), True)
    await stream(ws, 1.0, lambda t: wxyz(0, 0, 0), lambda t: (0.0, 0.015 * min(1.0, t), 0.0), True)
    await stream(ws, 0.5, lambda t: wxyz(0, 0, 0), lambda t: (0.0, 0.015, 0.0), False)
    print("translate_hold done")


@scenario("reset")
async def reset_scenario(ws):
    await stream(ws, 0.5, lambda t: wxyz(0, 0, 0.4), lambda t: (0, 0, 0), True)
    await ws.send(json.dumps({"type": "reset", "_id": 1}))
    print("reset response:", await asyncio.wait_for(ws.recv(), timeout=2.0))
    await asyncio.sleep(1.7)
    await ws.send(json.dumps({"type": "status", "_id": 2}))
    print("status:", await asyncio.wait_for(ws.recv(), timeout=2.0))


@scenario("drop_mid_motion")
async def drop_mid_motion(ws):
    await stream(ws, 0.8, lambda t: wxyz(0, 0.2 * t, 0), lambda t: (0, 0, 0), True)
    print("dropping connection mid-motion")
    await ws.close()


async def run(url, name):
    if name == "list":
        for k in sorted(SCENARIOS):
            print(k)
        return
    if name not in SCENARIOS:
        print(f"unknown scenario: {name}", file=sys.stderr)
        sys.exit(2)
    async with websockets.connect(url) as ws:
        await SCENARIOS[name](ws)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("scenario")
    p.add_argument("--url", default=DEFAULT_URL)
    args = p.parse_args()
    asyncio.run(run(args.url, args.scenario))


if __name__ == "__main__":
    main()
