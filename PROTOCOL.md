# Motion Controller UDP Protocol v4

Cross-repo contract between the ESP32 firmware and the Reachy Mini Python app.
When this document changes, update in the same change:

- `firmware/main/net_link.c`
- `reachy-mini-app/esp32_motion_controller/protocol.py`
- `reachy-mini-app/esp32_motion_controller/session.py`

**Major version 4 is required on both sides.** Mixed v3/v4 pairs ignore each other.
There is no compatibility adapter.

## Transport

| Item | Value |
|------|-------|
| Role | Python app binds UDP; ESP32 sends datagrams to that address |
| Port | `8766/udp` (HTTP status UI stays on `8766/tcp`) |
| Framing | one JSON object per datagram |
| Discovery | mDNS `_reachyctl._tcp.local.` (port 8766) plus `RMC2?` UDP probe |
| Fallback | `CONFIG_RMC_ROBOT_HOST` compiled override |
| Threat model | Trusted LAN only — no auth, plain UDP |

Port **8766** is intentional so this app can coexist with `spectacles_reachy_mini` on 8765.

There is no connection, handshake, ping/pong, or reconnect. A lost datagram costs one sample period. The next datagram supersedes it.

## Rates

| Path | Rate |
|------|------|
| ESP32 IMU fusion (Mahony) | 250 Hz (on device) |
| ESP32 → Python sample | 20 Hz fire-and-forget, latest-value |
| Python → ESP32 state reply | one datagram per received sample |
| Python control / SDK command loop | 20 Hz fixed-rate, one SDK call in flight |
| Streaming SDK speed lock | 86 °/s per-axis rotation, 30 mm/s positional slews |
| Appear/disappear SDK speed lock | 215 °/s per-axis rotation, 75 mm/s positional slews (2.5× streaming) |

The device must never queue catch-up samples. The host keeps only the latest valid sample.

## Frame limits

| Direction | Max UTF-8 bytes |
|-----------|-----------------|
| Device → host | 512 |
| Host → device | 512 |

Malformed, oversize, or non-finite numeric frames are dropped. Discovery probes are not JSON.

## Liveness

Derived, not negotiated:

| Side | Linked when |
|------|-------------|
| Device | last host reply younger than **1 s** |
| Host | last device datagram younger than **1 s** |

Boot shows a gray Starting… label until the first host reply, or for 15 s if none arrives. After that, a red DISCONNECTED label follows liveness (shown after 1.5 s unlinked). A 5 s unlink makes the device re-resolve the host.

## Discovery

ESP32 broadcasts `RMC2?` (and mDNS-browses `_reachyctl._tcp`). Host replies:

```
RMC2 <ipv4> 8766
```

## Messages: ESP32 → Python

### sample (20 Hz)

```json
{
  "pv": 4,
  "boot_id": "a1b2c3d4",
  "seq": 412,
  "q": [0.99, 0.01, -0.13, 0.02],
  "engaged": true,
  "gain": 1.0,
  "ready": true,
  "op": 3
}
```

| Field | Type | Units / notes |
|-------|------|---------------|
| `pv` | int | Must be `4` |
| `boot_id` | string | Opaque string unique per device reboot |
| `seq` | uint32 | Monotonic per boot; wraps; used for duplicate/reorder detection |
| `q` | `[w, x, y, z]` | Unit quaternion, body→world (gravity-aligned Mahony fusion). Host snapshots `q` at engage as IMU zero; subsequent tilt/yaw are relative to that pose. |
| `engaged` | bool | True while the BOOT toggle is on |
| `gain` | float | Motion multiplier in `[0.1, 2.0]` |
| `ready` | bool | False until gyro bias calibration succeeds |
| `op` | uint32, optional | Boot-scoped reset token. Omitted when idle. Repeated in every sample until the reply carries a terminal `op_status`. |

Host keeps only the latest valid sample. Duplicates (`seq == last`) and older reorder (`seq < last`, not a uint32 wrap) are dropped. A `boot_id` change reseeds clutch.

The device gates outbound `engaged` false while last host `mode` is `resetting` or `fault`.

### hello (diag only)

Sent every 2 s until the first host reply, and again while unlinked. Never gates sample flow.

```json
{
  "pv": 4,
  "type": "hello",
  "boot_id": "a1b2c3d4",
  "device": "esp32-reachy-ctl",
  "diag": {
    "rst": 1,
    "wifi_n": 0,
    "wifi_r": 0,
    "rssi": -52,
    "wifi_up": 1,
    "down_ms": 0,
    "send_ok": 0,
    "send_fail": 0,
    "send_ms": 0
  }
}
```

`diag` is omitted by loopback/fake clients. Hosts ignore unknown keys.

## Messages: Python → ESP32

One reply per received sample (and per hello). Same shape:

```json
{
  "pv": 4,
  "robot": true,
  "mode": "idle",
  "error": null,
  "op_ack": 3,
  "op_status": "completed"
}
```

| Field | Meaning |
|-------|---------|
| `robot` | Reachy Mini daemon / SDK is available (or log-only) |
| `mode` | `idle` \| `engaged` \| `resetting` \| `fault` |
| `error` | Optional short string when `mode` is `fault` |
| `op_ack` / `op_status` | Present while a reset token is in flight or cached. `op_status` is `accepted` \| `completed` \| `failed`. |

## Reset semantics

1. Device sets `op` to a new boot-scoped token and repeats it on every sample.
2. Host enters `mode=resetting`, force-disengages, and slews toward neutral at the same per-tick speed cap as streaming (`set_target` only).
3. Replies carry `op_ack` + `op_status=accepted` until arrival at neutral, then `completed` / `failed`. If still disengaged the host slews to the ducked rest pose.
4. Reset is idempotent: the same `(boot_id, op)` returns the cached outcome and never starts a second slew.
5. A packet gap does not cancel in-flight `set_target`.

## Motion policy

- Engage control is **rotation only**: the IMU stream drives roll/pitch/yaw. Head `x`/`y`/`z` stay at the clutch base pose while engaged. Host-owned appear/disappear/reset slews still move position.
- Appear and disappear run **only** on the BOOT button edge. A sample gap or unlink must not duck or rise. First app start may skip appear (posture unknown).
- If no valid sample arrives for **600 ms** and the controller is absent, the host force-disengages in place (holds the last pose). It does not run disappear.
- Every outbound command (streaming, appear, disappear, reset) is mapped through the last delivered pose: jumps **> 20 mm or 20°** are dropped; streaming and reset are capped at **86 °/s per axis** (roll, pitch, yaw independently) and **30 mm/s** positional slews per 50 ms tick; appear/disappear use **215 °/s** and **75 mm/s** (2.5×). Head yaw is clamped to **±150°** (30° of margin from the ±180° representation wrap) and the head–body yaw delta to **±65°**; those workspace limits are applied after the per-tick cap so an infeasible pair is never sent.
- The robot SDK is only ever called with `set_target` — never a blocking `goto_target`.
- A same-`boot_id` stream keeps `q_ref`, desired pose, button latch, and posture. A new boot ID or fault follows the normal reseed path.

## Appear / disappear

BOOT appear/disappear is owned by the host as speed-locked `set_target` slews
at 2.5× the streaming cap (215 °/s per-axis / 75 mm/s). No exclusive `goto_target`.

- **Disappear** (BOOT off only): slew to rest pose (`z = -0.05` m, `pitch = +5°`).
- **Appear** (BOOT on from ducked rest): slew to reset pose. Unknown posture
  (first connect or reseed) clutches in place and does not run appear.

## Axis convention

Device (landscape, screen facing user, USB + buttons facing down): `X` right, `Y` up,
`Z` out of screen. The screen *is* Reachy's face.

Robot head frame: `x` forward, `y` left, `z` up.

Python maps with the similarity transform `M R_rel M⁻¹` where

```
M = [[0, 0, 1],   # head_x ← device_z  (face)
     [1, 0, 0],   # head_y ← device_x  (screen's left)
     [0, 1, 0]]   # head_z ← device_y  (up)
```

On engage, Python snapshots device `q` as the clutch origin.
Roll/pitch/yaw are all relative to the engage pose (40° of board tilt spans ±25° of
head travel). Yaw is 1:1 with the board pan. Pitch and roll stay inside
the Stewart ±25° stop; only yaw unwraps past that. Yaw heading is the azimuth of
the board's **right edge** (device +X), so a nod cannot leak into yaw. Yaw itself
is a bounded linear axis in **±150°** (never ±180°, which is the IK wrap that
would slam the body motor). Past a stop it holds, heading tracking continues so a
long one-direction pan cannot alias and reverse, and it only returns when the
board turns back.

| Device motion | Head |
|---------------|------|
| tip top toward you (rot about +X) | pitch nose-down |
| turn screen toward your right (rot about +Y) | yaw same way |
| raise screen's right edge (rot about +Z) | roll same way |

## Cutover / rollback

Flash firmware and upgrade the Python app **together**.

| Pair | Result |
|------|--------|
| FW v4 + app v4 | Supported |
| Mixed v3/v4 | `pv` mismatch — frames dropped |
| FW v2 + app v4 | Device WebSocket client, host UDP — no traffic |
| FW v4 + app v2 | Device UDP, host WebSocket — no traffic |

Rollback: reflash previous firmware image and reinstall previous app wheel/commit as a pair.
