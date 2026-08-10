# Motion Controller WebSocket Protocol v2

Cross-repo contract between the ESP32 firmware and the Reachy Mini Python app.
When this document changes, update in the same change:

- `firmware/main/net_ws.c`
- `reachy-mini-app/esp32_motion_controller/protocol.py`
- `reachy-mini-app/esp32_motion_controller/session.py`

**Major version 2 is required on both sides.** Mixed v1/v2 pairs must fail visibly.
There is no compatibility adapter.

## Transport

| Item | Value |
|------|-------|
| Role | Python app is the server; ESP32 is the client |
| URL | `ws://<host>:8766/ws` |
| Framing | JSON text frames |
| Discovery | mDNS service `_reachyctl._tcp.local.` (port 8766) |
| Fallback | `CONFIG_RMC_ROBOT_HOST` compiled override |
| Threat model | Trusted LAN only — no auth, plain `ws://` |

Port **8766** is intentional so this app can coexist with `spectacles_reachy_mini` on 8765.

## Rates

| Path | Rate |
|------|------|
| ESP32 IMU fusion / ZUPT integration | 250 Hz (on device) |
| ESP32 → Python `sample` | 20 Hz fire-and-forget, latest-value |
| Python control / SDK command loop | 20 Hz fixed-rate, one SDK call in flight |

A blocked send may lower the telemetry rate. The device must never queue catch-up samples.
Host SDK latency may lower the command rate. The host must never accumulate sample backlog.

## Frame limits

| Direction | Max UTF-8 bytes |
|-----------|-----------------|
| Device → host | 512 |
| Host → device | 512 |

Malformed, oversize, or non-finite numeric frames are rejected. The host replies with
`error` when a request expected a response; fire-and-forget `sample` frames are dropped
silently after logging.

## Handshake

On WebSocket connect the device sends:

```json
{
  "type": "hello",
  "protocol_version": 2,
  "boot_id": "a1b2c3d4",
  "device": "esp32-reachy-ctl"
}
```

| Field | Notes |
|-------|-------|
| `protocol_version` | Integer major version; must be `2` |
| `boot_id` | Opaque string unique per device reboot (hex or decimal) |
| `device` | Informative identifier |

The host replies:

```json
{
  "type": "hello",
  "protocol_version": 2,
  "session_id": 7
}
```

If the host receives a missing/unsupported major version it closes the socket after:

```json
{
  "type": "error",
  "request_type": "hello",
  "message": "unsupported protocol_version"
}
```

Immediately after a successful hello the host pushes a `host_state` snapshot.

## Messages: ESP32 → Python

### `sample` (20 Hz, fire-and-forget)

```json
{
  "type": "sample",
  "boot_id": "a1b2c3d4",
  "seq": 412,
  "q": [0.99, 0.01, -0.13, 0.02],
  "p": [0.004, -0.011, 0.002],
  "engaged": true,
  "gain": 1.0,
  "ready": true
}
```

| Field | Type | Units / notes |
|-------|------|---------------|
| `boot_id` | string | Same as hello; host rejects mismatch for the session |
| `seq` | uint32 | Monotonic per boot; wraps; used for duplicate/reorder detection |
| `q` | `[w, x, y, z]` | Unit quaternion, body→world (gravity-aligned Mahony fusion) |
| `p` | `[x, y, z]` | metres; ZUPT-aided displacement in the **world** frame |
| `engaged` | bool | True only while the face is held (touch engage) |
| `gain` | float | Motion multiplier in `[0.1, 3.0]` |
| `ready` | bool | False until gyro bias calibration succeeds |

There is **no per-sample acknowledgement**. The host keeps only the latest valid sample.

While a reset is in progress the device may pause `sample` frames. The host treats a
300 ms gap (by host receipt time) as stale regardless.

### `reset` (on demand)

```json
{
  "type": "reset",
  "boot_id": "a1b2c3d4",
  "op_id": 3
}
```

| Field | Notes |
|-------|-------|
| `op_id` | Boot-scoped monotonic token; retries reuse the same token |

Responses use `reset_result` (below). Reset is idempotent: a second `reset` with the
same `(boot_id, op_id)` returns the cached outcome and never starts a second goto.

## Messages: Python → ESP32

### `host_state` (event-driven)

Sent on connect and whenever mode / robot readiness / error changes. Not polled.

```json
{
  "type": "host_state",
  "robot": true,
  "mode": "idle",
  "error": null
}
```

| Field | Meaning |
|-------|---------|
| `robot` | Reachy Mini daemon / SDK is available (or log-only) |
| `mode` | `idle` \| `engaged` \| `resetting` \| `fault` |
| `error` | Optional short string when `mode` is `fault` |

The device gates outbound `engaged` false while `mode` is `resetting` or `fault`.

### `reset_result`

```json
{
  "type": "reset_result",
  "boot_id": "a1b2c3d4",
  "op_id": 3,
  "status": "completed"
}
```

| `status` | Meaning |
|----------|---------|
| `accepted` | Reset started (optional early ack; may be omitted) |
| `completed` | SDK minjerk goto finished and pose rebased |
| `failed` | Reset could not run or SDK failed |

A replacement WebSocket after reconnect may re-query outcome by sending the same
`reset` token; the host returns the cached `reset_result` without re-running the goto
if that operation already finished.

### `error`

```json
{
  "type": "error",
  "request_type": "reset",
  "message": "..."
}
```

## Connection policy

- Single active controller session. A new WebSocket **replaces** a stale one (ESP
  auto-reconnect); the previous socket is closed. Session generation increments so
  callbacks from the old socket cannot mutate the active session.
- If no valid `sample` arrives for **300 ms** (host monotonic receipt time), the host
  force-disengages (commits clutch) and freezes the last safe command.
- On reconnect, the host seeds from the measured robot pose before accepting control.
  The first `engaged: true` after a seed / stale gap is a fresh rising edge.
- Sequence: ignore duplicates (`seq == last`); accept wrap only after a large backward
  jump consistent with uint32 wrap or a new `boot_id`/session.

## Reset semantics

1. Device may pause samples and sends `reset` with a new `op_id`.
2. Host enters `mode=resetting`, force-disengages, rebases clutch toward neutral intent.
3. Host runs blocking `goto_target(..., method="minjerk", duration=1.5)` off the event
   loop. No concurrent `set_target`.
4. Host reads measured pose, rebases baselines, pushes `host_state` (`idle`), and sends
   `reset_result` with `status=completed` or `failed`.
5. Disconnect during reset does **not** cancel the robot motion by dropping an executor
   future. Outcome is cached by `(boot_id, op_id)`.

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

`p` is world-frame. On engage, Python rotates `Δp` into the clutch reference
(device frame at engage) before applying `M`.

| Device motion | Head |
|---------------|------|
| tip top toward you (rot about +X) | pitch nose-down |
| turn screen toward your right (rot about +Y) | yaw same way |
| raise screen's right edge (rot about +Z) | roll same way |
| +X displacement (right) | +y |
| +Y displacement (up) | +z |
| +Z displacement (toward user) | +x |

## Cutover / rollback

Flash firmware and upgrade the Python app **together**.

| Pair | Result |
|------|--------|
| FW v2 + app v2 | Supported |
| FW v1 + app v2 | Hello fails / no `controller_state` handler — non-functional |
| FW v2 + app v1 | App does not understand `sample`/`hello` — non-functional |

Rollback: reflash previous firmware image and reinstall previous app wheel/commit as a pair.
