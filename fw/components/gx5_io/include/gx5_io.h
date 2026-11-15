// gx5_io — TCA9555 expander, MPU-6050 motion wake, PCF8563 RTC. All tolerate
// missing hardware. The 3V3-PSM rail must be up before gx5_i2c_init().
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <time.h>

void gx5_i2c_init(void);                  // expander, RTC -> system clock, IMU motion wake
void gx5_i2c_scan(char *out, size_t size); // addresses that answer (on-board and harness devices)
void gx5_exp_set(uint16_t mask, bool on); // GX5_EXP_* outputs (gx5.h)
bool gx5_charging(void);
void gx5_imu_arm(void);                   // clear a latched motion interrupt
bool gx5_imu_read(int mg[3], int *temp_dc, int ddps[3]); // acceleration (mg), temperature (0.1 degC), rotation (0.1 deg/s)
void gx5_imu_gyro(bool on);               // gyro on + driving-event detection (posts GX5_EV_MOTION), or standby
void gx5_rtc_set(time_t utc);
