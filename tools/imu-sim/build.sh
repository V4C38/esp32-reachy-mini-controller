#!/bin/sh
# Compile the host IMU acceptance harness against the firmware integrator
# so the two cannot diverge.
set -e
here=$(cd "$(dirname "$0")" && pwd)
fw="$here/../../firmware/main"
mkdir -p "$here/out"
cc -O2 -std=c11 -Wall -Wextra -I "$fw" \
  "$here/imu_sim_main.c" "$fw/imu_integrate.c" -lm \
  -o "$here/out/imu_sim"
echo "$here/out/imu_sim"
