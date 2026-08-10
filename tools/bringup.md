# Hardware bring-up checklist

Prerequisites: flashed firmware with `sdkconfig.local` WiFi, Reachy Mini Lite on USB, daemon running, Motion Controller app on the Mac.

## 1. Link

- [ ] Board joins WiFi (serial: `net_wifi` connected)
- [ ] mDNS finds `_reachyctl._tcp` or `CONFIG_RMC_ROBOT_HOST` override works
- [ ] Face leaves disconnected (closed eyes) → idle open eyes
- [ ] `status` shows `robot: true`

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

Raw QMI8658 axes on this board, physically measured (see the comment above
`IMU_MAP_*` in `firmware/main/config.h`): raw +X = up the native-portrait
screen, raw +Y = toward the USB edge, raw +Z = into the case. Hence
`X = ax, Y = -ay, Z = -az`.

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

Rotation remap lives in `controller_state.py` (`DEV_TO_HEAD` + similarity
transform). Displacement remap is the same matrix after rotating world-frame
`Δp` into the engage reference.

## 4. Clutch / ZUPT feel

- [ ] Touch-hold engages after ~120 ms; release holds pose
- [ ] Move-and-stop: displacement lands and holds (no creep)
- [ ] Stationary while engaged: no drift beyond deadband
- [ ] Pure tilt does not shove the head in translation
- [ ] If needed, record `CONFIG_RMC_TRACE` and tune thresholds in `imu_integrate.c`

## 5. Safety / range

- [ ] Ellipsoid roll/pitch radii (~25°) feel safe on Lite
- [ ] Translation clamp (~20 mm head / ~80 mm hand) matches comfortable workspace
- [ ] Reset returns to neutral and ignores engage while busy
- [ ] Double-tap opens settings; gain persists across reboot (NVS)

## 6. Log-only first

Before the robot, run the app with `--log-only` and confirm `controller_state`
packets decode with sensible `q`/`p` while moving the board. Host acceptance:

```bash
bash tools/imu-sim/build.sh && tools/imu-sim/out/imu_sim --self-test
```
