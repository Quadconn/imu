#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "imu.h"

void app_main(void) {
    uint32_t sample_count = 0;
    AccelerationMps2 accel = {0};
    AngularVelocityDps ang_vel = {0};

    if (imu_init())
    {
        while (1)
        {
            bool is_accel_sample_ready = false;
            if (imu_is_accel_data_ready() && imu_get_accel_sample(&accel))
            {
                is_accel_sample_ready = true;

            }

            bool is_gyro_sample_ready = false;
            if (imu_is_gyro_data_ready() && imu_get_gyro_sample(&ang_vel))
            {

                is_gyro_sample_ready = true;
            }

            if (is_accel_sample_ready && is_gyro_sample_ready) {
                printf("IMU [%ld]: accel(%f, %f, %f), gyro(%f, %f, %f)\n",
                        sample_count,
                        accel.x, accel.y, accel.z,
                        ang_vel.x, ang_vel.y, ang_vel.z);
            }

            sample_count++;

            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    imu_deinit();
}
