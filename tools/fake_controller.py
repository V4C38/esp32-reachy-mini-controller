#!/usr/bin/env python3
"""Play scripted protocol-v2 sample sequences against the Motion Controller app."""

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

BOOT_ID = "fake-boot"


def wxyz(roll: float, pitch: float, yaw: float) -> list[float]:
    q = R.from_euler("xyz", [roll, pitch, yaw], degrees=False).as_quat()
    return [float(q[3]), float(q[0]), float(q[1]), float(q[2])]


def sample(seq: int, q, p, engaged: bool, gain: float = 1.0, ready: bool = True) -> dict:
    return {
        "type": "sample",
        "boot_id": BOOT_ID,
        "seq": seq,
        "q": q,
        "p": list(p),
        "engaged": engaged,
        "gain": gain,
        "ready": ready,
    }


async def stream(ws, duration, q_fn, p_fn, engaged, hz=20.0, seq0=0):
    dt = 1.0 / hz
    t0 = time.monotonic()
    seq = seq0
    while time.monotonic() - t0 < duration:
        t = time.monotonic() - t0
        await ws.send(json.dumps(sample(seq, q_fn(t), p_fn(t), engaged)))
        seq += 1
        await asyncio.sleep(dt)
    return seq


async def handshake(ws) -> None:
    await ws.send(
        json.dumps(
            {
                "type": "hello",
                "protocol_version": 2,
                "boot_id": BOOT_ID,
                "device": "fake-controller",
            }
        )
    )
    hello = json.loads(await asyncio.wait_for(ws.recv(), timeout=2.0))
    assert hello.get("type") == "hello", hello
    # host_state follows
    st = json.loads(await asyncio.wait_for(ws.recv(), timeout=2.0))
    assert st.get("type") == "host_state", st


async def run_engage_rotate_release(url: str) -> None:
    async with websockets.connect(url) as ws:
        await handshake(ws)
        seq = await stream(ws, 0.4, lambda t: wxyz(0, 0, 0), lambda t: (0, 0, 0), True)
        seq = await stream(
            ws, 0.8, lambda t: wxyz(0, 0, 0.3 * min(1.0, t / 0.8)), lambda t: (0, 0, 0), True, seq0=seq
        )
        await ws.send(json.dumps(sample(seq, wxyz(0, 0, 0.3), (0, 0, 0), False)))
        await ws.send(json.dumps({"type": "reset", "boot_id": BOOT_ID, "op_id": 1}))
        resp = json.loads(await asyncio.wait_for(ws.recv(), timeout=3.0))
        print("reset:", resp)
        # May receive accepted then completed / host_state
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            try:
                msg = json.loads(await asyncio.wait_for(ws.recv(), timeout=0.5))
            except TimeoutError:
                break
            print("msg:", msg)
            if msg.get("type") == "reset_result" and msg.get("status") in {"completed", "failed"}:
                break


SCRIPTS = {
    "engage_rotate_release": run_engage_rotate_release,
}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("script", choices=sorted(SCRIPTS))
    ap.add_argument("--url", default="ws://127.0.0.1:8766/ws")
    args = ap.parse_args()
    asyncio.run(SCRIPTS[args.script](args.url))


if __name__ == "__main__":
    main()
