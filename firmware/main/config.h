#pragma once

#include "sdkconfig.h"

#define RMC_WS_PORT CONFIG_RMC_ROBOT_PORT
#define RMC_SEND_HZ 20
#define RMC_IMU_HZ 250

/* Hold pose: screen toward user, USB + buttons facing down.
 * Device frame after remap: +X right, +Y up, +Z out of screen (screen = face).
 *
 * Raw QMI8658 axes on this board, physically measured (accel reads +9.8 on
 * whichever axis points up):
 *   flat on desk, screen up  -> raw (0, 0, -9.8)   up = raw -Z
 *   hold pose (USB down)     -> raw (0, -9.8, 0)   up = raw -Y
 *   upright, USB port right  -> raw (+9.8, 0, 0)   up = raw +X
 * So: device X (right) = raw +X, device Y (up) = raw -Y,
 *     device Z (out of screen) = raw -Z. det = +1.
 *
 * Engage logs a warning if remapped accel is not ~ (0, +9.8, 0).
 * Bring-up checks are in tools/bringup.md §2. */
#define IMU_MAP_X(ax, ay, az) (ax)
#define IMU_MAP_Y(ax, ay, az) (-(ay))
#define IMU_MAP_Z(ax, ay, az) (-(az))

#define UI_BRIGHTNESS_IDLE 45
#define UI_BRIGHTNESS_ENGAGED 100
#define UI_BRIGHTNESS_DISCONNECTED 30

#define TOUCH_ENGAGE_MS 120
#define TOUCH_DOUBLE_TAP_MS 350

#define GAIN_DEFAULT 1.0f
#define GAIN_MIN 0.1f
#define GAIN_MAX 3.0f
