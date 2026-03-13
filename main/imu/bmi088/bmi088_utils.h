#ifndef BMI088_UTILS_H
#define BMI088_UTILS_H

#include <stdint.h>

/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
float lsb_to_mps2(int16_t val, int8_t g_range, uint8_t bit_width);

/*!
 * @brief This function converts lsb to rad per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
float lsb_to_rps(int16_t val, float dps, uint8_t bit_width);


void bmi08_error_codes_print_result(const char api_name[], int8_t rslt);

#endif
