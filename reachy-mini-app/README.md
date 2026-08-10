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
  - websocket
  - imu
---

# ESP32 Motion Controller

WebSocket bridge for an ESP32 handheld IMU controller that drives Reachy Mini's head with clutch-style relative motion, IK-safe clamping, antenna idle animation, and body follow only when head exceeds the neck yaw threshold.

See the [repository README](https://github.com/V4C38/esp32-reachy-mini-controller) and [`PROTOCOL.md`](../PROTOCOL.md).

## Local install

```bash
pip install -e .
python -m esp32_motion_controller.main
```

WebSocket: `ws://<host>:8766/ws`


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

Ensure README frontmatter includes those tags before upload.

The excludes matter: `packages.find` would otherwise pick up a stray `build/lib/`
tree as a second copy of the package, and `.pyc` files built by a different
Python end up installed into the daemon's 3.12 venv.
