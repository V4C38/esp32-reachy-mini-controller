# Firmware resource baselines

Status: **firmware image builds on ESP-IDF 5.5**; boot/WiFi heap captured 2026-08-13 on Waveshare ESP32-S3-Touch-AMOLED-1.8 (`diag` logs). See [`idf_size.txt`](idf_size.txt) for the static size snapshot from CI.

## Hard constraints (v1, must not regress)

| Item | Value | Source |
|---|---|---|
| Target | ESP32-S3 | `sdkconfig.defaults` |
| Flash | 16 MB | `CONFIG_ESPTOOLPY_FLASHSIZE_16MB` |
| PSRAM | Octal 80 MHz, fetch instr/rodata | `CONFIG_SPIRAM_*` |
| Internal DMA reserve | 102400 bytes | `CONFIG_SPIRAM_MALLOC_RESERVE_INTERNAL` |
| LVGL strip height | 50 | `CONFIG_BSP_DISPLAY_LVGL_BUF_HEIGHT` |
| QSPI DMA chunk | 16 KB max transfer, queue depth 2 | BSP fork |
| LVGL core | 1 | BSP `task_affinity` |
| IMU task | core 0, prio 4, stack 4096 | `main.c` |
| App/UDP task | core 0, prio 5, stack 8192 | `main.c` |
| Sample TX buffer | 360 B stack | `net_link.c` |
| WiFi PS | `WIFI_PS_NONE` | `net_wifi.c` |

## Measured table (fill on hardware)

| State | Free internal | Free DMA | Largest DMA block | PSRAM used | imu HWM | app HWM | Notes |
|---|---:|---:|---:|---:|---:|---:|---|
| Boot before WiFi | 232767 | 224979 | 98304 | ~2.7 KB | | | `diag` boot; DMA reserve 100 KB intact |
| Linked idle | 148143 | 140355 | 98304 | ~1.71 MB | 2016 | 4040 | `link_up`; HWM is remaining stack bytes; no flush timeouts |
| Settings open | | | | | | | |
| Engaged stream | | | | | | | |
| Reconnect churn | | | | | | | USB-Serial/JTAG capture can stall WiFi. Idle `/api/status` soak with serial closed. |
| Display recovery | | | | | | | |

Acceptance: every filled cell for v4 must meet or improve the v3 measurement within noise; zero new display flush failures; no application-owned heap alloc on the 20 Hz sample path.
