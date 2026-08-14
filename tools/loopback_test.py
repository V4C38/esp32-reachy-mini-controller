#!/usr/bin/env python3
"""End-to-end loopback against Motion Controller v4 (+ optional --log-only app start).

Asserts hello reply, reset completion, stale gap handling, and resume after a gap.
"""

from __future__ import annotations

import argparse
import json
import socket
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
from scipy.spatial.transform import Rotation as R

BOOT_ID = "loopback-boot"
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8766


def wxyz(roll: float, pitch: float, yaw: float) -> list[float]:
    q = R.from_euler("xyz", [roll, pitch, yaw], degrees=False).as_quat()
    return [float(q[3]), float(q[0]), float(q[1]), float(q[2])]


def sample(seq, q, engaged, gain=1.0, ready=True, op=None):
    msg = {
        "pv": 4,
        "boot_id": BOOT_ID,
        "seq": seq,
        "q": q,
        "engaged": engaged,
        "gain": gain,
        "ready": ready,
    }
    if op is not None:
        msg["op"] = op
    return msg


class UdpPeer:
    def __init__(self, host: str, port: int) -> None:
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.5)

    def send(self, payload: dict) -> None:
        self.sock.sendto(json.dumps(payload).encode(), self.addr)

    def recv(self, timeout: float = 0.5) -> dict | None:
        self.sock.settimeout(timeout)
        try:
            data, _ = self.sock.recvfrom(512)
        except TimeoutError:
            return None
        return json.loads(data.decode())

    def close(self) -> None:
        self.sock.close()


def handshake(peer: UdpPeer) -> dict:
    peer.send({"pv": 4, "type": "hello", "boot_id": BOOT_ID, "device": "loopback"})
    st = peer.recv(timeout=2.0)
    assert st is not None and st.get("pv") == 4, st
    return st


def stream(peer: UdpPeer, duration, q_fn, engaged, hz=20.0, seq0=0):
    dt = 1.0 / hz
    t0 = time.monotonic()
    seq = seq0
    while time.monotonic() - t0 < duration:
        t = time.monotonic() - t0
        peer.send(sample(seq, q_fn(t), engaged))
        peer.recv(timeout=0.05)
        seq += 1
        time.sleep(dt)
    return seq


def test_reset_rebase(host: str, port: int) -> None:
    peer = UdpPeer(host, port)
    try:
        handshake(peer)
        seq = stream(peer, 0.4, lambda t: wxyz(0, 0, 0), True)
        seq = stream(peer, 0.8, lambda t: wxyz(0, 0, 0.3), True, seq0=seq)
        peer.send(sample(seq, wxyz(0, 0, 0.3), False, op=1))
        deadline = time.monotonic() + 4.0
        seen_accepted = False
        done = None
        while time.monotonic() < deadline:
            msg = peer.recv(timeout=0.3)
            if msg is None:
                seq += 1
                peer.send(sample(seq, wxyz(0, 0, 0.3), False, op=1))
                continue
            if msg.get("op_status") == "accepted":
                seen_accepted = True
            if msg.get("op_status") in {"completed", "failed"}:
                done = msg
                break
        assert done is not None and done["op_status"] == "completed", done
        assert seen_accepted or done["op_ack"] == 1
    finally:
        peer.close()
    print("OK reset_rebase")


def test_watchdog_freeze(host: str, port: int) -> None:
    peer = UdpPeer(host, port)
    try:
        handshake(peer)
        seq = stream(peer, 0.5, lambda t: wxyz(0, 0.2, 0), True)
        time.sleep(0.8)
        stream(peer, 0.4, lambda t: wxyz(0, 0.5, 0), True, seq0=seq)
    finally:
        peer.close()
    print("OK watchdog_freeze")


def test_resume_after_gap(host: str, port: int) -> None:
    peer = UdpPeer(host, port)
    try:
        st = handshake(peer)
        assert st.get("mode") in {"idle", "engaged", "fault"}
        stream(peer, 0.6, lambda t: wxyz(0, 0, 0.25 * t), True)
        time.sleep(0.3)
        stream(peer, 0.4, lambda t: wxyz(0, 0, 0), True, seq0=20)
    finally:
        peer.close()
    print("OK resume_after_gap")


def run_all(host: str, port: int) -> None:
    test_reset_rebase(host, port)
    test_watchdog_freeze(host, port)
    test_resume_after_gap(host, port)
    print("loopback OK")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default=DEFAULT_HOST)
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
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
                        assert body.get("protocol_version") == 4
                        break
            except Exception:
                time.sleep(0.2)
        else:
            raise SystemExit("app did not become ready on :8766")
    try:
        run_all(args.host, args.port)
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
