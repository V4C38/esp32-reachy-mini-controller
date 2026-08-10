# Firmware resource baselines

Status: **firmware image builds on ESP-IDF 5.5**; runtime heap/HWM table still needs board capture. See [`idf_size.txt`](idf_size.txt) for the static size snapshot from CI.

## Hard constraints (v1, must not regress)

| Item | Value | Source |
|---|---|---|
| Target | ESP32-S3 | `sdkconfig.defaults` |
| Flash | 16 MB | `CONFIG_ESPTOOLPY_FLASHSIZE_16MB` |
| PSRAM | Octal 80 MHz, fetch instr/rodata | `CONFIG_SPIRAM_*` |
| Internal DMA reserve | 102400 bytes | `CONFIG_SPIRAM_MALLOC_RESERVE_INTERNAL` |
| LVGL strip height | 50 | `CONFIG_BSP_DISPLAY_LVGL_BUF_HEIGHT` |
| QSPI DMA chunk | 16 KB max transfer | BSP fork |
| LVGL core | 1 | BSP `task_affinity` |
| IMU task | core 0, prio 4, stack 4096 | `main.c` |
| App/WS task | core 0, prio 5, stack 8192 | `main.c` |
| WS TX buffer (app) | 320 B stack | `net_ws.c` |
| WS client buffer | 2048 B | `net_ws.c` |
| WiFi PS | `WIFI_PS_NONE` | `net_wifi.c` |

## Measured table (fill on hardware)

| State | Free internal | Free DMA | Largest DMA block | PSRAM used | imu HWM | app HWM | Notes |
|---|---:|---:|---:|---:|---:|---:|---|
| Boot before WiFi | | | | | | | |
| Linked idle | | | | | | | |
| Settings open | | | | | | | |
| Engaged stream | | | | | | | |
| Reconnect churn | | | | | | | |
| Display recovery | | | | | | | |

Acceptance: every filled cell for v2 must meet or improve the v1 measurement within noise; zero new display flush failures; no application-owned heap alloc on the 20 Hz sample path.
