# Hardware bring-up checklist (protocol v4)

Prerequisites: flashed **v4** firmware with `sdkconfig.local` WiFi, Reachy Mini Lite on USB, daemon running, Motion Controller **v4** app on the Mac.

## 1. Link

- [ ] Board joins WiFi (serial: `net_wifi` connected)
- [ ] mDNS finds `_reachyctl._tcp` or `CONFIG_RMC_ROBOT_HOST` override works
- [ ] Serial shows `link up`
- [ ] Gray Starting… at bottom center during boot; label hides once the host replies (red DISCONNECTED only if the app never answers, or after a later unlink)
- [ ] Eyes stay closed until BOOT engage (not tied to link)
- [ ] `/api/status` shows `protocol_version: 4` and `robot: true`

## 2. IMU_MAP axis verification

Hold the board with USB + buttons facing down, screen facing you. BOOT
snapshots that grab as IMU zero. Boot prints `mapped accel [...]` after
calibration; also enable `CONFIG_RMC_TRACE` or watch the engage log line
(`engage accel=[...] gyro=[...]`).

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

Engage and move slowly. The screen *is* Reachy's face. Control is
**rotation only** — translating the board must not shove the head.

| Device motion | Expected head |
|---|---|
| Tip top toward you (rot about +X) | Head pitch nose-down |
| Turn screen toward your right (rot about +Y) | Head yaw matching |
| Raise screen's right edge (rot about +Z) | Head roll matching |

Rotation remap lives in `control.py` (`DEV_TO_HEAD` + similarity transform).
Roll and pitch use a 40° board window from the **engage snapshot** to span
the head's ±25°. Yaw is adaptive (1:1 short pans, quadratic ease-in to 2:1
by 50° of pan). The
on-device Motion Multiplier slider (0.1×–2×) scales all rotation axes equally.

## 4. Clutch feel

- [ ] BOOT press toggles engage (eyes open); second press clutches (eyes closed)
- [ ] PWR is unused for motion (do not use it — 6 s hold powers off)
- [ ] Engaging at a tilted pose does **not** move the head (that grab is zero)
- [ ] Clutching and re-engaging re-zeros to the new grab; the prior head pose is held
- [ ] In-place turn while engaged yaws the head; turning while clutched does not
- [ ] Past a yaw stop the head holds; it does not slew to the opposite side
- [ ] Translating the board while engaged does not move head `x`/`y`/`z`
- [ ] 600 ms WiFi gap force-disengages without a snap on resume

## 5. Safety / reset

- [ ] Roll/pitch radii (~25°) feel safe on Lite
- [ ] Reset returns to neutral; samples pause / engage ignored while `mode=resetting`
- [ ] Double-tap opens settings; gain persists across reboot (NVS)
- [ ] Reconnect mid-hold does not whip the head

## 6. Resource / display soak

- [ ] No black screen after `idf.py flash` (face appears within ~2 s)
- [ ] `idf.py monitor --no-reset` / `python tools/capture.py` attach without blanking the panel
- [ ] Confirm the WS link with `/api/status` while the serial port is **closed**. USB-Serial/JTAG capture (even `--no-reset`) can stall `transport_poll_write` and look like reconnects. `python tools/soak_link.py --seconds 90 --require-connected` polls at 1 Hz and prints `last_diag` / close code on any flip.
- [ ] Engaged stream 10+ minutes: no flush timeout storms, no regular WS reconnects
- [ ] Optional: film the static face at 30 fps / 1/60 s shutter — motion-tracking bands should be gone (panel PWM scan may still show; see README *Camera banding when filming*)
- [ ] Head never exceeds 86 °/s per axis / 30 mm/s on streaming `set_target`
- [ ] Record heap/HWM into `tools/baselines/firmware_resources.md`

## 7. Log-only first

Before the robot, run the app with `--log-only` and confirm `sample` packets
decode while moving the board. Host acceptance:

```bash
bash tools/imu-sim/build.sh && tools/imu-sim/out/imu_sim --self-test
python tools/loopback_test.py --start-app
```
