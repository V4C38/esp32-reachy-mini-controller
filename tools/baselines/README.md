# v1 baselines (freeze before v2)

Captured against commit `47eb67f` (v1).

## Behavior trajectories

[`v1_trajectories.json`](v1_trajectories.json) holds clutch, gain, body-follow, stale-disengage, and ellipsoid outputs from the v1 Python pipeline. Translation fixtures were dropped in protocol v4 (rotation-only). Later golden tests compare against what remains.

## Firmware resource envelope

Measure on the Waveshare ESP32-S3-Touch-AMOLED-1.8 before a firmware cutover:

| State | Metrics |
|---|---|
| Boot before WiFi | `idf.py size`, free internal heap, DMA heap, largest DMA block, PSRAM |
| Linked idle | same + task high-water marks (`imu`, `app`, LVGL, UDP) |
| Settings open | same |
| Sustained engaged streaming | same + send fail count, IMU deadline misses |
| Reconnect churn | same |
| Display flush recovery | flush timeout streak, free DMA |

Hard constraints (must not regress):

- `CONFIG_SPIRAM_MALLOC_RESERVE_INTERNAL=102400`
- QSPI `max_transfer_sz` capped at 16 KB
- LVGL buffers in PSRAM; LVGL pinned to core 1
- `WIFI_PS_NONE`
- Application stacks: `imu` 4096, `app` 8192

Record values in [`firmware_resources.md`](firmware_resources.md) when board + IDF are available:

```bash
. ~/esp/esp-idf/export.sh
cd firmware && idf.py size
# Attach without resetting the panel: idf.py monitor --no-reset
# or: python tools/capture.py 30 /dev/cu.usbmodem101
```
