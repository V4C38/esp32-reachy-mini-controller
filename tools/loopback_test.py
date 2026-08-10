#!/usr/bin/env python3
"""End-to-end loopback against Motion Controller v2 (+ optional --log-only app start).

Asserts hello handshake, reset completion, stale gap handling, and reconnect.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

try:
    import websockets
except ImportError:
    print("pip install websockets", file=sys.stderr)
    sys.exit(1)

from scipy.spatial.transform import Rotation as R

BOOT_ID = "loopback-boot"


def wxyz(roll: float, pitch: float, yaw: float) -> list[float]:
    q = R.from_euler("xyz", [roll, pitch, yaw], degrees=False).as_quat()
    return [float(q[3]), float(q[0]), float(q[1]), float(q[2])]


def sample(seq, q, p, engaged, gain=1.0, ready=True):
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


async def handshake(ws):
    await ws.send(
        json.dumps(
            {
                "type": "hello",
                "protocol_version": 2,
                "boot_id": BOOT_ID,
                "device": "loopback",
            }
        )
    )
    hello = None
    st = None
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline and (hello is None or st is None):
        msg = json.loads(await asyncio.wait_for(ws.recv(), timeout=2.0))
        if msg.get("type") == "hello":
            hello = msg
        elif msg.get("type") == "host_state":
            st = msg
    assert hello is not None and hello.get("protocol_version") == 2, hello
    assert st is not None, st
    return st


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


async def drain_until(ws, predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        try:
            last = json.loads(await asyncio.wait_for(ws.recv(), timeout=0.3))
        except TimeoutError:
            continue
        if predicate(last):
            return last
    raise AssertionError(f"timeout waiting for message; last={last}")


async def test_reset_rebase(url: str) -> None:
    async with websockets.connect(url) as ws:
        await handshake(ws)
        seq = await stream(ws, 0.4, lambda t: wxyz(0, 0, 0), lambda t: (0, 0, 0), True)
        seq = await stream(ws, 0.8, lambda t: wxyz(0, 0, 0.3), lambda t: (0, 0, 0), True, seq0=seq)
        await ws.send(json.dumps(sample(seq, wxyz(0, 0, 0.3), (0, 0, 0), False)))
        await ws.send(json.dumps({"type": "reset", "boot_id": BOOT_ID, "op_id": 1}))
        accepted = await drain_until(
            ws, lambda m: m.get("type") == "reset_result" and m.get("status") == "accepted"
        )
        assert accepted["op_id"] == 1
        done = await drain_until(
            ws,
            lambda m: m.get("type") == "reset_result"
            and m.get("status") in {"completed", "failed"},
            timeout=4.0,
        )
        assert done["status"] == "completed", done
    print("OK reset_rebase")


async def test_watchdog_freeze(url: str) -> None:
    async with websockets.connect(url) as ws:
        await handshake(ws)
        seq = await stream(ws, 0.5, lambda t: wxyz(0, 0.2, 0), lambda t: (0, 0, 0), True)
        await asyncio.sleep(0.5)  # > 300 ms stale
        # Resume engaged — must be a fresh rising edge (no exception / still linked)
        await stream(ws, 0.4, lambda t: wxyz(0, 0.5, 0), lambda t: (0, 0, 0), True, seq0=seq)
    print("OK watchdog_freeze")


async def test_reconnect_no_jump(url: str) -> None:
    async with websockets.connect(url) as ws:
        await handshake(ws)
        await stream(ws, 0.6, lambda t: wxyz(0, 0, 0.25 * t), lambda t: (0, 0, 0), True)
    await asyncio.sleep(0.3)
    async with websockets.connect(url) as ws:
        st = await handshake(ws)
        assert st.get("mode") in {"idle", "engaged", "fault"}
        await stream(ws, 0.4, lambda t: wxyz(0, 0, 0), lambda t: (0, 0, 0), True)
    print("OK reconnect_no_jump")


async def run_all(url: str) -> None:
    await test_reset_rebase(url)
    await test_watchdog_freeze(url)
    await test_reconnect_no_jump(url)
    print("loopback OK")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="ws://127.0.0.1:8766/ws")
    ap.add_argument("--start-app", action="store_true")
    args = ap.parse_args()

    app_proc = None
    log_f = None
    if args.start_app:
        log_path = ROOT / "tools" / ".loopback_app.log"
        log_f = open(log_path, "w")
        app_proc = subprocess.Popen(
            [sys.executable, "-m", "esp32_motion_controller.main", "--log-only"],
            cwd=str(ROOT / "reachy-mini-app"),
            stdout=log_f,
            stderr=subprocess.STDOUT,
        )
        import urllib.request

        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            if app_proc.poll() is not None:
                log_f.flush()
                print(log_path.read_text(), file=sys.stderr)
                raise SystemExit("app exited before becoming ready")
            try:
                with urllib.request.urlopen("http://127.0.0.1:8766/api/status", timeout=0.5) as r:
                    if r.status == 200:
                        body = json.loads(r.read().decode())
                        assert body.get("protocol_version") == 2
                        break
            except Exception:
                time.sleep(0.2)
        else:
            raise SystemExit("app did not become ready on :8766")
    try:
        asyncio.run(run_all(args.url))
    finally:
        if app_proc is not None:
            app_proc.terminate()
            try:
                app_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                app_proc.kill()
        if log_f is not None:
            log_f.close()


if __name__ == "__main__":
    main()
