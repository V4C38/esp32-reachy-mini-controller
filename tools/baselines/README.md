# v1 baselines (freeze before v2)

Captured against commit `47eb67f` (v1).

## Behavior trajectories

[`v1_trajectories.json`](v1_trajectories.json) records deterministic clutch, translation, gain, body-follow, stale-disengage, and ellipsoid outputs from the v1 Python pipeline. v2 golden tests compare against these fixtures.

## Firmware resource envelope

Hardware measurements must be taken on the Waveshare ESP32-S3-Touch-AMOLED-1.8 before accepting a firmware cutover. Capture at least:

| State | Metrics |
|---|---|
| Boot before WiFi | `idf.py size`, free internal heap, DMA heap, largest DMA block, PSRAM |
| Linked idle | same + task high-water marks (`imu`, `app`, LVGL, WS client) |
| Settings open | same |
| Sustained engaged streaming | same + send fail count, IMU deadline misses |
| Reconnect churn | same |
| Display flush recovery | flush timeout streak, free DMA |

Known v1 hard constraints (must not regress):

- `CONFIG_SPIRAM_MALLOC_RESERVE_INTERNAL=102400`
- QSPI `max_transfer_sz` capped at 16 KB
- LVGL buffers in PSRAM; LVGL pinned to core 1
- `WIFI_PS_NONE`
- Application stacks: `imu` 4096, `app` 8192

Record measured values in [`firmware_resources.md`](firmware_resources.md) when board + IDF are available:

```bash
. ~/esp/esp-idf/export.sh
cd firmware && idf.py size
# With monitor attached after flash, dump heap via the resource log hook.
```
