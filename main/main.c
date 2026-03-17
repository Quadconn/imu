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

#include "esp_timer.h"
void app_main(void) {
    uint32_t sample_count = 0;
    AccelerationMps2 accel = {0};
    AngularVelocityRps ang_vel = {0};
    Quaternion q = QUAT_IDENTITY;
    Quaternion q_delta = {0};
    Euler e = {0};

    if (imu_init())
    {



int64_t t_prev = esp_timer_get_time();

while (1) {
    // NOTE: The angle * dt has large effect on output if dt is not accurate.
    if (imu_is_gyro_data_ready() && imu_get_gyro_sample(&ang_vel)) {
        int64_t t_now = esp_timer_get_time();
        float dt = (t_now - t_prev) * 1e-6f;  // microseconds → seconds
        t_prev = t_now;

        float angle = magnitude3f(ang_vel.x, ang_vel.y, ang_vel.z) * dt;

        if (angle > 1e-6f) {
            quat_from_angle_axis(&q_delta, angle, ang_vel.x, ang_vel.y, ang_vel.z);
            quat_multiply(&q, &q, &q_delta);
        }

        if (sample_count % 100 == 0) {
            quat_normalize(&q);
            quat_to_euler(&e, &q);
            printf("e(%f, %f, %f)\n",
                e.roll  * (180.0f / (float)M_PI),
                e.pitch * (180.0f / (float)M_PI),
                e.yaw   * (180.0f / (float)M_PI));
        }

        sample_count++;
    }
}



        //while (1)
        //{
        //    //bool is_accel_sample_ready = imu_is_accel_data_ready() && imu_get_accel_sample(&accel);

        //    bool is_gyro_sample_ready = imu_is_gyro_data_ready() && imu_get_gyro_sample(&ang_vel);

        //    //if (is_accel_sample_ready && is_gyro_sample_ready) {
        //    if (is_gyro_sample_ready) {

        //        float angle = magnitude3f(ang_vel.x, ang_vel.y, ang_vel.z) * 0.0015179113539769279;

        //        quat_from_angle_axis(&q_delta, angle, ang_vel.x, ang_vel.y, ang_vel.z);

        //        quat_multiply(&q, &q, &q_delta);

        //        //printf("q(%f, %f, %f, %f)\n", q.w, q.x, q.y, q.z);

        //        if (sample_count % 100 == 0) {
        //            quat_to_euler(&e, &q);

        //            printf("e(%f, %f, %f)\n", e.roll * (180.0f / M_PI), e.pitch * (180.0f / M_PI), e.yaw * (180.0f / M_PI));

        //        }


        //        //printf("IMU [%ld]: accel(%f, %f, %f), gyro(%f, %f, %f)\n",
        //        //        sample_count,
        //        //        accel.x, accel.y, accel.z,
        //        //        ang_vel.x, ang_vel.y, ang_vel.z);
        //    }

        //    sample_count++;
        //}


        // Add this temporarily to measure real ODR

//int64_t t_start = esp_timer_get_time();  // microseconds
//uint32_t gyro_samples = 0;
//
//while (gyro_samples < 1000) {
//    if (imu_is_gyro_data_ready() && imu_get_gyro_sample(&ang_vel)) {
//        gyro_samples++;
//    }
//}
//
//int64_t t_end = esp_timer_get_time();
//float actual_odr = 1000.0f / ((t_end - t_start) / 1e6f);
//printf("Actual gyro ODR: %.1f Hz\n", actual_odr);
    }

    imu_deinit();
}
