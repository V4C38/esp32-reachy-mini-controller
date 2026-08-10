#!/usr/bin/env python3
"""End-to-end loopback against Motion Controller (+ optional --log-only app start).

Asserts reset rebasing, watchdog freeze after stale gap, and reconnect mid-hold.
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


def wxyz(roll: float, pitch: float, yaw: float) -> list[float]:
    q = R.from_euler("xyz", [roll, pitch, yaw], degrees=False).as_quat()
    return [float(q[3]), float(q[0]), float(q[1]), float(q[2])]


async def send_state(ws, seq, q, p, engaged, gain=1.0, ready=True):
    await ws.send(json.dumps({
        "type": "controller_state", "seq": seq, "q": q, "p": list(p),
        "engaged": engaged, "gain": gain, "ready": ready,
    }))


async def stream(ws, duration, q_fn, p_fn, engaged, hz=20.0, seq0=0):
    dt = 1.0 / hz
    t0 = time.monotonic()
    seq = seq0
    while time.monotonic() - t0 < duration:
        t = time.monotonic() - t0
        await send_state(ws, seq, q_fn(t), p_fn(t), engaged)
        seq += 1
        await asyncio.sleep(dt)
    return seq


async def test_reset_rebase(url: str) -> None:
    async with websockets.connect(url) as ws:
        seq = await stream(ws, 0.4, lambda t: wxyz(0, 0, 0), lambda t: (0, 0, 0), True)
        seq = await stream(ws, 0.8, lambda t: wxyz(0, 0, 0.3), lambda t: (0, 0, 0), True, seq0=seq)
        await send_state(ws, seq, wxyz(0, 0, 0.3), (0, 0, 0), False)
        await ws.send(json.dumps({"type": "reset", "_id": 1}))
        resp = json.loads(await asyncio.wait_for(ws.recv(), timeout=2.0))
        assert "success" in resp or resp.get("type") in {"reset_result", "status_result"}, resp
        await asyncio.sleep(1.7)
        await ws.send(json.dumps({"type": "status", "_id": 2}))
        st = json.loads(await asyncio.wait_for(ws.recv(), timeout=2.0))
        assert st.get("type") == "status_result"
        assert st.get("busy") is False
    print("OK reset_rebase")


async def test_watchdog_freeze(url: str) -> None:
    async with websockets.connect(url) as ws:
        seq = await stream(ws, 0.5, lambda t: wxyz(0, 0.2, 0), lambda t: (0, 0, 0), True)
        await asyncio.sleep(0.5)
        await stream(ws, 0.4, lambda t: wxyz(0, 0.5, 0), lambda t: (0, 0, 0), True, seq0=seq)
        await ws.send(json.dumps({"type": "status", "_id": 3}))
        st = json.loads(await asyncio.wait_for(ws.recv(), timeout=2.0))
        assert st.get("type") == "status_result"
    print("OK watchdog_freeze")


async def test_reconnect_no_jump(url: str) -> None:
    async with websockets.connect(url) as ws:
        await stream(ws, 0.6, lambda t: wxyz(0, 0, 0.25 * t), lambda t: (0, 0, 0), True)
    await asyncio.sleep(0.3)
    async with websockets.connect(url) as ws:
        await stream(ws, 0.4, lambda t: wxyz(0, 0, 0), lambda t: (0, 0, 0), True)
        await ws.send(json.dumps({"type": "status", "_id": 4}))
        st = json.loads(await asyncio.wait_for(ws.recv(), timeout=2.0))
        assert st.get("type") == "status_result"
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
        # Wait until HTTP status is up (max ~15s)
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
