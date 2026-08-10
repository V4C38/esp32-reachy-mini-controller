# Motion Controller WebSocket Protocol

Cross-repo contract between the ESP32 firmware and the Reachy Mini Python app.
When this document changes, update in the same change:

- `firmware/main/net_ws.c`
- `reachy-mini-app/esp32_motion_controller/ws_handler.py`

## Transport

| Item | Value |
|------|-------|
| Role | Python app is the server; ESP32 is the client |
| URL | `ws://<host>:8766/ws` |
| Framing | JSON text frames |
| Discovery | mDNS service `_reachyctl._tcp.local.` (port 8766) |
| Fallback | `CONFIG_RMC_ROBOT_HOST` compiled override |

Port **8766** is intentional so this app can coexist with `spectacles_reachy_mini` on 8765.

## Rates

| Path | Rate |
|------|------|
| ESP32 IMU fusion / ZUPT integration | 250 Hz (on device) |
| ESP32 → Python `controller_state` | 20 Hz |
| Python apply / LERP loop | ~30 Hz |
| Python → Reachy Mini daemon `set_target` | 20 Hz |

## Messages: ESP32 → Python

### `controller_state` (20 Hz, fire-and-forget)

```json
{
  "type": "controller_state",
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
| `seq` | int | Monotonic packet counter |
| `q` | `[w, x, y, z]` | Unit quaternion, body→world (gravity-aligned Mahony fusion) |
| `p` | `[x, y, z]` | metres; ZUPT-aided displacement in the **world** frame |
| `engaged` | bool | True only while the face is held (touch engage) |
| `gain` | float | Motion multiplier in `[0.1, 3.0]` (scales rotation; also multiplies translation) |
| `ready` | bool | False until gyro bias calibration succeeds |

Python refuses to engage while `ready` is false.

### `reset` (on demand)

```json
{ "type": "reset", "_id": 7 }
```

Response:

```json
{ "type": "reset_result", "success": true, "_id": 7 }
```

Triggers a 1.5 s minjerk `goto` to the neutral pose, then rebases the clutch.

### `status` (on demand)

```json
{ "type": "status", "_id": 8 }
```

## Messages: Python → ESP32

### `status_result`

```json
{
  "type": "status_result",
  "connected": true,
  "robot": true,
  "busy": false,
  "_id": 8
}
```

| Field | Meaning |
|-------|---------|
| `connected` | WebSocket session is live |
| `robot` | Reachy Mini daemon / SDK is available |
| `busy` | Reset `goto` in progress; Python ignores `engaged` |

### `error`

```json
{
  "type": "error",
  "request_type": "reset",
  "message": "...",
  "_id": 7
}
```

## Connection policy

- Single controller only: a second WebSocket client is refused with an explicit error.
- If no `controller_state` arrives for 300 ms, Python treats the controller as disengaged and freezes the target pose.
- On reconnect, the first `engaged: true` is a fresh rising edge and re-snapshots the clutch reference.

## Axis convention

Device (landscape, screen facing user, USB + buttons facing down): `X` right, `Y` up, `Z` out of screen.
The screen *is* Reachy's face.

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
