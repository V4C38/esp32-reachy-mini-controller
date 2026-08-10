#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root/reachy-mini-app"
if [[ ! -d .venv ]]; then python3 -m venv .venv; fi
# shellcheck disable=SC1091
source .venv/bin/activate
pip install -q -e ".[dev]"
pkill -f "esp32_motion_controller.main" 2>/dev/null || true
python ../tools/loopback_test.py --start-app
echo "loopback OK"
