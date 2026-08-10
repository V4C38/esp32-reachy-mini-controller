# Hardware bring-up checklist (protocol v2)

Prerequisites: flashed **v2** firmware with `sdkconfig.local` WiFi, Reachy Mini Lite on USB, daemon running, Motion Controller **v2** app on the Mac.

## 1. Link

- [ ] Board joins WiFi (serial: `net_wifi` connected)
- [ ] mDNS finds `_reachyctl._tcp` or `CONFIG_RMC_ROBOT_HOST` override works
- [ ] Serial shows `host hello ok (protocol 2)`
- [ ] Face leaves disconnected (closed eyes) → idle open eyes
- [ ] `/api/status` shows `protocol_version: 2` and `robot: true`

## 2. IMU_MAP axis verification

Hold the board with USB + buttons facing down, screen facing you. Boot prints
`mapped accel [...]` after calibration; also enable `CONFIG_RMC_TRACE` or watch
the engage log line (`engage accel=[...] gyro=[...]`).

| Check | Expected remapped reading |
|---|---|
| Boot flat on the table, screen up | boot log accel `~(0, 0, +9.81)` |
| Idle pose (screen toward you, USB down) | accel `~(0, +9.81, 0)` |
| Tip top of board toward you | `gyro_x > 0` |
| Turn screen toward your right | `gyro_y > 0` |
| Raise the screen's right edge | `gyro_z > 0` |

If idle accel is not dominated by `+Y`, engage logs
`IMU_MAP sanity: expected accel ~ (0, +9.8, 0)` — fix `IMU_MAP_*` in
`firmware/main/config.h`.

## 3. Device → head mapping

Engage and move slowly. The screen *is* Reachy's face.

| Device motion | Expected head |
|---|---|
| Tip top toward you (rot about +X) | Head pitch nose-down |
| Turn screen toward your right (rot about +Y) | Head yaw matching |
| Raise screen's right edge (rot about +Z) | Head roll matching |
| Translate device +X (right) | Head +y |
| Translate device +Y (up) | Head +z |
| Translate device +Z (toward you) | Head +x |

Rotation remap lives in `control.py` (`DEV_TO_HEAD` + similarity transform).

## 4. Clutch / ZUPT feel

- [ ] Touch-hold engages after ~120 ms; release holds pose
- [ ] Move-and-stop: displacement lands and holds (no creep)
- [ ] Stationary while engaged: no drift beyond deadband
- [ ] Pure tilt does not shove the head in translation
- [ ] 300 ms WiFi gap force-disengages without a snap on resume

## 5. Safety / reset

- [ ] Ellipsoid roll/pitch radii (~25°) feel safe on Lite
- [ ] Translation clamp (~20 mm head / ~80 mm hand) matches comfortable workspace
- [ ] Reset returns to neutral; samples pause / engage ignored while `mode=resetting`
- [ ] Double-tap opens settings; gain persists across reboot (NVS)
- [ ] Reconnect mid-hold does not whip the head

## 6. Resource / display soak

- [ ] No black screen across repeated `idf.py monitor` USB RTS cycles
- [ ] Engaged stream 10+ minutes: no flush timeout storms
- [ ] Record heap/HWM into `tools/baselines/firmware_resources.md`

## 7. Log-only first

Before the robot, run the app with `--log-only` and confirm `sample` packets
decode while moving the board. Host acceptance:

```bash
bash tools/imu-sim/build.sh && tools/imu-sim/out/imu_sim --self-test
python tools/loopback_test.py --start-app
```
