#ifndef BMI088_ESP32_INTERFACE
#define BMI088_ESP32_INTERFACE


#include <stdint.h>

#include "bmi08_defs.h"

bool bmi088_esp32_interface_init(struct bmi08_dev* bmi088);
void bmi08_error_codes_print_result(const char api_name[], int8_t rslt);

#endif
