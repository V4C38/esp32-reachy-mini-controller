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

echo "==> Firmware build (optional)"
if command -v idf.py >/dev/null 2>&1; then
  ( cd firmware && idf.py build )
elif [[ -f "${HOME}/esp/esp-idf/export.sh" ]]; then
  # Avoid ambient venv click pollution; export in a clean subshell
  env -i HOME="$HOME" USER="$USER" PATH="/usr/bin:/bin:/usr/sbin:/sbin:/opt/homebrew/bin:${HOME}/.espressif/tools" TERM="${TERM:-}"     bash -lc '. "$HOME/esp/esp-idf/export.sh" >/dev/null && cd "'"$root"'/firmware" && idf.py build'
else
  echo "IDF not available; skipping firmware build"
fi

echo "CI OK"
