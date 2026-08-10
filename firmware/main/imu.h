#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "imu_integrate.h"

esp_err_t imu_init(i2c_master_bus_handle_t bus);
bool imu_ready(void);
bool imu_still(void);

/* Run one 250 Hz sample (call from a single IMU task only). */
bool imu_update(float dt);

/* Thread-safe snapshot of the latest integrator state. */
void imu_get_state(imu_integrate_state_t *out);

/* Latest remapped accel (m/s^2) and gyro (rad/s) in the device frame. */
void imu_get_raw_mapped(float accel[3], float gyro[3]);

/* True if remapped accel is consistent with the intended hold
 * (screen toward user, USB+buttons down ⇒ gravity along +Y). */
bool imu_gravity_sane(void);

void imu_reset_displacement(void);
