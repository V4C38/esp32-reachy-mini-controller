#!/usr/bin/env python3
"""Play scripted protocol-v4 sample sequences against the Motion Controller app."""

from __future__ import annotations

import argparse
import asyncio
import json
import socket
import sys
import time

from scipy.spatial.transform import Rotation as R

BOOT_ID = "fake-boot"
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8766


def wxyz(roll: float, pitch: float, yaw: float) -> list[float]:
    q = R.from_euler("xyz", [roll, pitch, yaw], degrees=False).as_quat()
    return [float(q[3]), float(q[0]), float(q[1]), float(q[2])]


def sample(seq: int, q, engaged: bool, gain: float = 1.0, ready: bool = True, op=None) -> dict:
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


async def stream(peer: UdpPeer, duration, q_fn, engaged, hz=20.0, seq0=0):
    dt = 1.0 / hz
    t0 = time.monotonic()
    seq = seq0
    while time.monotonic() - t0 < duration:
        t = time.monotonic() - t0
        peer.send(sample(seq, q_fn(t), engaged))
        seq += 1
        await asyncio.sleep(dt)
    return seq


def handshake(peer: UdpPeer) -> dict:
    peer.send({"pv": 4, "type": "hello", "boot_id": BOOT_ID, "device": "fake-controller"})
    st = peer.recv(timeout=2.0)
    assert st is not None and st.get("pv") == 4, st
    return st


async def run_engage_rotate_release(host: str, port: int) -> None:
    peer = UdpPeer(host, port)
    try:
        handshake(peer)
        seq = await stream(peer, 0.4, lambda t: wxyz(0, 0, 0), True)
        seq = await stream(
            peer, 0.8, lambda t: wxyz(0, 0, 0.3 * min(1.0, t / 0.8)), True, seq0=seq
        )
        peer.send(sample(seq, wxyz(0, 0, 0.3), False, op=1))
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            msg = peer.recv(timeout=0.5)
            if msg is None:
                peer.send(sample(seq, wxyz(0, 0, 0.3), False, op=1))
                seq += 1
                continue
            print("msg:", msg)
            if msg.get("op_status") in {"completed", "failed"}:
                break
    finally:
        peer.close()


SCRIPTS = {
    "engage_rotate_release": run_engage_rotate_release,
}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("script", choices=sorted(SCRIPTS))
    ap.add_argument("--host", default=DEFAULT_HOST)
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    args = ap.parse_args()
    asyncio.run(SCRIPTS[args.script](args.host, args.port))


if __name__ == "__main__":
    main()
