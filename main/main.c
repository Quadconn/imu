#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "imu.h"
#include "quaternion.h"

float magnitude3f(float x, float y, float z) {
    return sqrtf((x * x) + (y * y) + (z * z));
}

void app_main(void) {
    uint32_t sample_count = 0;
    AccelerationMps2 accel = {0};
    AngularVelocityRps ang_vel = {0};
    Quaternion q = QUAT_IDENTITY;
    Quaternion q_delta;

    if (imu_init())
    {
        while (1)
        {
            bool is_accel_sample_ready = imu_is_accel_data_ready() && imu_get_accel_sample(&accel);

            bool is_gyro_sample_ready = imu_is_gyro_data_ready() && imu_get_gyro_sample(&ang_vel);

            if (is_accel_sample_ready && is_gyro_sample_ready) {

                float angle = magnitude3f(ang_vel.x, ang_vel.y, ang_vel.z) * 1E-3f;

                quat_from_angle_axis(&q_delta, angle, ang_vel.x, ang_vel.y, ang_vel.z);

                quat_multiply(&q, &q, &q_delta);

                printf("q(%f, %f, %f, %f)\n", q.w, q.x, q.y, q.z);

                //printf("IMU [%ld]: accel(%f, %f, %f), gyro(%f, %f, %f)\n",
                //        sample_count,
                //        accel.x, accel.y, accel.z,
                //        ang_vel.x, ang_vel.y, ang_vel.z);
            }

            sample_count++;

            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    imu_deinit();
}
