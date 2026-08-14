---
title: ESP32 Motion Controller — Reachy Mini
emoji: 🕹️
colorFrom: gray
colorTo: yellow
sdk: static
pinned: false
short_description: ESP32 handheld motion controller for Reachy Mini
tags:
  - reachy_mini
  - reachy_mini_python_app
  - esp32
  - udp
  - imu
---

# ESP32 Motion Controller

Reachy Mini app that receives protocol v4 UDP samples from the handheld ESP32. Engage snapshots IMU zero; roll/pitch/yaw stay clutch-relative (rotation only). The app owns IK-safe clamping, hard yaw stops, antenna idle, and body follow.

See the [repository README](https://github.com/V4C38/esp32-reachy-mini-controller) and [`PROTOCOL.md`](../PROTOCOL.md). Firmware and app must both speak v4.

## Local install

```bash
pip install -e .
python -m esp32_motion_controller.main
```

UDP: `<host>:8766`

Live Space: https://huggingface.co/spaces/V4C38/esp32_motion_controller

## Publish to Hugging Face

Tags required: `reachy_mini`, `reachy_mini_python_app`.

```bash
# from reachy-mini-app/
hf auth login
hf upload YOUR_USER/esp32_motion_controller . --repo-type space \
  --exclude ".venv/*" \
  --exclude "build/*" \
  --exclude "dist/*" \
  --exclude "*.egg-info/*" \
  --exclude "**/__pycache__/*" \
  --exclude "*.pyc" \
  --exclude ".pytest_cache/*" \
  --exclude ".DS_Store"
```

Keep those tags in the README frontmatter. The excludes matter: `packages.find` would otherwise pick up a stray `build/lib/` as a second copy of the package, and `.pyc` files from another Python end up in the daemon's 3.12 venv.
