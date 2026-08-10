# ESP32 Reachy Mini Motion Controller

<img src="assets/reachy_face.png" alt="Reachy face on the AMOLED display" width="320">

Hold a [Waveshare ESP32-S3-Touch-AMOLED-1.8](https://www.waveshare.com/esp32-s3-touch-amoled-1.8.htm) with the screen facing you and the USB port down — the screen *is* Reachy's face. Touch and hold, and [Reachy Mini](https://www.pollen-robotics.com/reachy-mini/)'s head copies how you tilt, turn, and move the device. Lift your finger to clutch, reposition your hand, and continue.

## What this is

This repo is a hardware motion controller for Reachy Mini, split across the board and the robot:

- **Custom firmware** for the Waveshare board — IMU fusion (Mahony + ZUPT displacement), an LVGL face that mirrors your motion, and a WiFi WebSocket client that streams sensor samples at 20 Hz
- **A Reachy Mini Python app** that receives samples through a versioned protocol, runs one pure control reducer, and owns a single fixed-rate SDK command loop (clutch mapping, workspace safety, antenna idle animation, body follow)
- **Host-side test harnesses** — IMU integrator simulation, fake controller, loopback tests, CI, v1 trajectory baselines

The ESP32 owns *how the device moved*. Python owns *what the robot should do*. The wire contract is **protocol v2** in [`PROTOCOL.md`](PROTOCOL.md). Firmware and app must be upgraded together.

## How it works

```
ESP32 (250 Hz IMU → 20 Hz sample)  →  Motion Controller app (:8766)
                                          ↓
                                 one 20 Hz control / SDK owner
                                          ↓
                                 Reachy Mini daemon (Lite / USB)
```

- **Motion** — the onboard QMI8658 runs through a Mahony filter for attitude and a ZUPT integrator for displacement. Axis conventions and bring-up checks live in [`tools/bringup.md`](tools/bringup.md).
- **Clutch mapping** — on engage, the app latches a reference pose; relative device rotation maps 1:1 onto the head (tip forward → head pitches down), displacement maps into head translation.
- **Safety** — the app velocity-clamps every command it streams to the daemon and fits targets into the Stewart platform's workspace ellipsoid, so the head can never snap.
- **Face UI** — the display shows Reachy's face: idle when linked, animated while engaged, eyes closed when disconnected. Double-tap opens settings (gain 0.1×–3×, reset pose).

## Running it

### 1. The Python app

```bash
# Start the Reachy Mini Desktop App / daemon first (robot on USB)
cd reachy-mini-app
python -m venv .venv && source .venv/bin/activate
pip install -e ".[dev]"
python -m esp32_motion_controller.main
```

Or install **Motion Controller** from the Reachy Mini Desktop App — published at [V4C38/esp32_motion_controller](https://huggingface.co/spaces/V4C38/esp32_motion_controller).

### 2. WiFi credentials

```bash
cp firmware/sdkconfig.local.example firmware/sdkconfig.local
# edit: CONFIG_RMC_WIFI_SSID / CONFIG_RMC_WIFI_PASSWORD  (gitignored)
```

The board finds the app via mDNS (`_reachyctl._tcp`); override with `CONFIG_RMC_ROBOT_HOST` if needed.

### 3. Build and flash

Flash from a plain shell — **don't do this with another Python venv active** (ESP-IDF's export picks up whatever `python3` is first on `PATH` and refuses to activate if that environment's packages conflict with IDF's constraints):

```bash
. ~/esp/esp-idf/export.sh          # ESP-IDF v5.5.5
cd firmware
idf.py set-target esp32s3          # first time only
idf.py -p /dev/cu.usbmodem101 flash monitor
```

Keeping `monitor` attached after flashing is the proper way to verify a flash: you should see `UI ready`, the IMU calibration line `mapped accel [...]` (≈ `[0 0 +9.8]` when the board lies flat on the desk), `host hello ok (protocol 2)`, and the face should appear within two seconds.

### Black screen (known failure mode)

If the AMOLED goes black and only recovers after a reset (USB RTS / board reset / power cycle), check the serial log for:

```text
E (…) spi_master: setup_dma_priv_buffer(…): Failed to allocate priv TX buffer
E (…) lcd_panel.io.spi: panel_io_spi_tx_color(…): spi transmit (queue) color failed
E (…) co5300_spi: panel_co5300_draw_bitmap(…): send color data failed
```

**Cause:** LVGL framebuffers live in PSRAM, so each QSPI flush copies a strip into an *internal* DMA bounce buffer. The SPI bus was configured with a full-frame `max_transfer_sz`, so those bounce allocs were ~70 KB+. After WiFi + the WebSocket stack came up, internal DMA heap was exhausted (`setup_dma_priv_buffer` failed), `draw_bitmap` never queued a transfer, the flush completion callback never ran, and the UI froze on a black panel until reboot.

**Fix (already in this tree):**
- Cap QSPI `max_transfer_sz` at 16 KB so bounce buffers stay small
- Raise `CONFIG_SPIRAM_MALLOC_RESERVE_INTERNAL` to 100 KB so DMA heap survives WiFi bring-up
- Keep LVGL buffers in PSRAM; do **not** enable `psram_dma_direct` here
- Serialize brightness `tx_param` with the LVGL lock
- Bounded flush wait recovers / restarts if a transfer never completes

A related failure: the panel lights at boot, then goes black while the firmware keeps running (mDNS / WiFi logs, no SPI errors). That happens when the CO5300 loses Sleep-Out / Display-On / GRAM / brightness after USB-UART **RTS/DTR** resets and WiFi RF bring-up. Firmware reasserts Display-On after WiFi comes up, periodically while linked, and with a full brightness + redraw keepalive while offline.

### 4. Hold it right

1. Screen toward you, **USB + side buttons facing down**.
2. Touch and hold to engage. Tip / turn / roll the board — the head copies that transform. Push the board and the head translates the same way.
3. Release to clutch. Double-tap for settings.

Untethered use needs a **3.7V MX1.25 LiPo** on the board; USB alone dies when unplugged.

## Protocol v2 cutover / rollback

| Pair | Result |
|------|--------|
| FW v2 + app v2 | Supported |
| Mixed v1/v2 | Non-functional (hello / message type mismatch) |

Rollback: reflash the previous firmware image and reinstall the previous app commit as a pair. Resource baselines: [`tools/baselines/`](tools/baselines/).

## Testing

```bash
# Unit tests + host IMU sim + loopback + firmware build (when IDF present)
./tools/run-ci.sh

# Loopback only
python tools/loopback_test.py --start-app
```

## Layout

| Path | Contents |
|---|---|
| `firmware/` | The ESP-IDF project: IMU, face UI, WiFi + WebSocket client |
| `reachy-mini-app/` | Motion Controller Reachy Mini app (protocol v2) |
| `tools/` | Serial capture, IMU sim, fake controller, baselines, bring-up, CI |
| `assets/` | Source artwork (Reachy face) |
| `PROTOCOL.md` | WebSocket JSON contract (v2) |

## License

[MIT](LICENSE)
