#include "bmi088_utils.h"

#include <stdint.h>
#include <math.h>
#include <stdio.h>

#include "bmi08_defs.h"


/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH  (9.80665f)


float lsb_to_mps2(int16_t val, int8_t g_range, uint8_t bit_width) {
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}


float lsb_to_rps(int16_t val, float dps, uint8_t bit_width) {
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return ((dps / (half_scale)) * (val)) * (M_PI / 180.0f);
}

// TODO DR: Using since driver examples make a lot of usage of it but potentially
// will adjust so printing doesn't happen directly in this module 
void bmi08_error_codes_print_result(const char api_name[], int8_t rslt)
{
    if (rslt != BMI08_OK)
    {
        printf("%s\t", api_name);
        if (rslt == BMI08_E_NULL_PTR)
        {
            printf("Error [%d] : Null pointer\n", rslt);
        }
        else if (rslt == BMI08_E_COM_FAIL)
        {
            printf("Error [%d] : Communication failure\n", rslt);
        }
        else if (rslt == BMI08_E_DEV_NOT_FOUND)
        {
            printf("Error [%d] : Device not found\n", rslt);
        }
        else if (rslt == BMI08_E_OUT_OF_RANGE)
        {
            printf("Error [%d] : Out of Range\n", rslt);
        }
        else if (rslt == BMI08_E_INVALID_INPUT)
        {
            printf("Error [%d] : Invalid input\n", rslt);
        }
        else if (rslt == BMI08_E_CONFIG_STREAM_ERROR)
        {
            printf("Error [%d] : Config stream error\n", rslt);
        }
        else if (rslt == BMI08_E_RD_WR_LENGTH_INVALID)
        {
            printf("Error [%d] : Invalid Read write length\n", rslt);
        }
        else if (rslt == BMI08_E_INVALID_CONFIG)
        {
            printf("Error [%d] : Invalid config\n", rslt);
        }
        else if (rslt == BMI08_E_FEATURE_NOT_SUPPORTED)
        {
            printf("Error [%d] : Feature not supported\n", rslt);
        }
        else if (rslt == BMI08_W_FIFO_EMPTY)
        {
            printf("Warning [%d] : FIFO empty\n", rslt);
        }
        else
        {
            printf("Error [%d] : Unknown error code\n", rslt);
        }
    }
}
