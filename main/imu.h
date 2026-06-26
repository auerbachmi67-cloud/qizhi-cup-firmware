#pragma once
#include <stdint.h>
void imu_init(void);
void imu_calibrate(void);
float imu_get_gyro_z(void);
float imu_get_accel_z(void);
