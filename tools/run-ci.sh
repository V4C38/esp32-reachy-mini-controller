#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root"

echo "==> Python unit tests"
if [[ -d reachy-mini-app/.venv ]]; then
  # shellcheck disable=SC1091
  source reachy-mini-app/.venv/bin/activate
fi
python3 -m pip install -q -e "reachy-mini-app[dev]"
( cd reachy-mini-app && python3 -m pytest ../tools/tests -q )

echo "==> Host IMU integrator sim"
bash tools/imu-sim/build.sh
tools/imu-sim/out/imu_sim --self-test

echo "==> Protocol v2 loopback"
pkill -f "esp32_motion_controller.main" 2>/dev/null || true
# Prefer the app venv interpreter so websockets/scipy resolve.
if [[ -x reachy-mini-app/.venv/bin/python ]]; then
  reachy-mini-app/.venv/bin/python tools/loopback_test.py --start-app
else
  python3 tools/loopback_test.py --start-app
fi

echo "==> Firmware build (optional)"
idf_py_env=""
if [[ -d "${HOME}/.espressif/python_env/idf5.5_py3.14_env" ]]; then
  idf_py_env="${HOME}/.espressif/python_env/idf5.5_py3.14_env"
elif [[ -d "${HOME}/.espressif/python_env/idf5.5_py3.9_env" ]]; then
  idf_py_env="${HOME}/.espressif/python_env/idf5.5_py3.9_env"
fi
if command -v idf.py >/dev/null 2>&1; then
  ( cd firmware && idf.py build && idf.py size | tee ../tools/baselines/idf_size.txt )
elif [[ -f "${HOME}/esp/esp-idf/export.sh" ]]; then
  # Avoid ambient venv click pollution; export in a clean subshell
  env -i HOME="$HOME" USER="$USER" PATH="/usr/bin:/bin:/usr/sbin:/sbin:/opt/homebrew/bin:${HOME}/.espressif/tools" TERM="${TERM:-}" \
    IDF_PYTHON_ENV_PATH="${idf_py_env}" \
    bash -lc '. "$HOME/esp/esp-idf/export.sh" >/dev/null && cd "'"$root"'/firmware" && idf.py build && idf.py size | tee ../tools/baselines/idf_size.txt'
else
  echo "IDF not available; skipping firmware build"
fi

echo "CI OK"
