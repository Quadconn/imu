#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>


typedef struct {
    float x;
    float y;
    float z;
} AccelerationMps2;

typedef struct {
    float x;
    float y;
    float z;
} AngularVelocityDps;

bool imu_init();
bool imu_deinit();
bool imu_is_accel_data_ready();
bool imu_is_gyro_data_ready();
bool imu_get_accel_sample(AccelerationMps2* out);
bool imu_get_gyro_sample(AngularVelocityDps* out);

#endif
