#include "imu.h"

// Driver
#include "bmi08.h"
#include "bmi08_defs.h"
#include "bmi08x.h"

// Integration
#include "bmi088_esp32_interface.h"
#include "bmi088_utils.h"


/*! @brief This structure containing relevant bmi08 info */
static struct bmi08_dev bmi08dev;

/*! @brief variable to hold the bmi08 accel data */
static struct bmi08_sensor_data bmi08_accel;

/*! @brief variable to hold the bmi08 gyro data */
static struct bmi08_sensor_data bmi08_gyro;

/*! bmi08 accel int config */
static struct bmi08_accel_int_channel_cfg accel_int_config;

/*! bmi08 gyro int config */
static struct bmi08_gyro_int_channel_cfg gyro_int_config;


// Static Declarations
static int8_t init_bmi08();
static int8_t enable_bmi08_interrupt();
static int8_t disable_bmi08_interrupt();


bool imu_init() {
    int8_t rslt;

    if (!bmi088_esp32_interface_init(&bmi08dev)) {
        return false;
    }

    rslt = init_bmi08();
    bmi08_error_codes_print_result("init_bmi08", rslt);

    if (rslt == BMI08_OK) {
        /* Enable data ready interrupts */
        rslt = enable_bmi08_interrupt();
        bmi08_error_codes_print_result("enable_bmi08_interrupt", rslt);
    }

    return (rslt == BMI08_OK);
}


bool imu_deinit() {
    int8_t rslt;
    rslt = disable_bmi08_interrupt();
    bmi08_error_codes_print_result("disable_bmi08_interrupt", rslt);

    return (rslt == BMI08_OK); 
}


bool imu_is_accel_data_ready() {
    int8_t rslt;
    uint8_t status;

    if (bmi08dev.accel_cfg.power != BMI08_ACCEL_PM_ACTIVE) {
        return false;
    }

    rslt = bmi08a_get_data_int_status(&status, &bmi08dev);
    bmi08_error_codes_print_result("bmi08a_get_data_int_status", rslt);

    return (rslt == BMI08_OK) && (bool)(status & BMI08_ACCEL_DATA_READY_INT);
}


bool imu_is_gyro_data_ready() {
    int8_t rslt;
    uint8_t status;

    if (bmi08dev.gyro_cfg.power != BMI08_GYRO_PM_NORMAL) {
        return false;
    }

    rslt = bmi08g_get_data_int_status(&status, &bmi08dev);
    bmi08_error_codes_print_result("bmi08g_get_data_int_status", rslt);

    return (rslt == BMI08_OK) && (bool)(status & BMI08_GYRO_DATA_READY_INT);
}


bool imu_get_accel_sample(AccelerationMps2* out) {
    int8_t rslt;

    if (bmi08dev.accel_cfg.power != BMI08_ACCEL_PM_ACTIVE) {
        return false;
    }

    rslt = bmi08a_get_data(&bmi08_accel, &bmi08dev);
    bmi08_error_codes_print_result("bmi08a_get_data", rslt);

    out->x = lsb_to_mps2(bmi08_accel.x, 24, 16);
    out->y = lsb_to_mps2(bmi08_accel.y, 24, 16);
    out->z = lsb_to_mps2(bmi08_accel.z, 24, 16);

    return (rslt == BMI08_OK);
}


bool imu_get_gyro_sample(AngularVelocityRps* out) {
    int8_t rslt;

    if (bmi08dev.gyro_cfg.power != BMI08_GYRO_PM_NORMAL) {
        return false;
    }

    rslt = bmi08g_get_data(&bmi08_gyro, &bmi08dev);
    bmi08_error_codes_print_result("bmi08g_get_data", rslt);

    out->x = lsb_to_rps(bmi08_gyro.x, 250.0f, 16);
    out->y = lsb_to_rps(bmi08_gyro.y, 250.0f, 16);
    out->z = lsb_to_rps(bmi08_gyro.z, 250.0f, 16);

    return (rslt == BMI08_OK);
}


static int8_t init_bmi08(void)
{
    int8_t rslt;

    rslt = bmi08xa_init(&bmi08dev);
    bmi08_error_codes_print_result("bmi08xa_init", rslt);

    if (rslt == BMI08_OK)
    {
        rslt = bmi08g_init(&bmi08dev);
        bmi08_error_codes_print_result("bmi08g_init", rslt);
    }

    if (rslt == BMI08_OK)
    {
        // NOTE: This is needed or else loading the config file will fail
        // Reset the accelerometer
        rslt = bmi08a_soft_reset(&bmi08dev);
        bmi08_error_codes_print_result("bmi08a_soft_reset", rslt);
    }

    if (rslt == BMI08_OK)
    {
        rslt = bmi08a_load_config_file(&bmi08dev);
        bmi08_error_codes_print_result("bmi08a_load_config_file", rslt);
    }

    if (rslt == BMI08_OK)
    {
        bmi08dev.accel_cfg.odr = BMI08_ACCEL_ODR_1600_HZ;

        if (bmi08dev.variant == BMI085_VARIANT)
        {
            bmi08dev.accel_cfg.range = BMI085_ACCEL_RANGE_16G;
        }
        else if (bmi08dev.variant == BMI088_VARIANT)
        {
            bmi08dev.accel_cfg.range = BMI088_ACCEL_RANGE_24G;
        }

        bmi08dev.accel_cfg.power = BMI08_ACCEL_PM_ACTIVE; /*user_accel_power_modes[user_bmi088_accel_low_power]; */
        bmi08dev.accel_cfg.bw = BMI08_ACCEL_BW_NORMAL; /* Bandwidth and OSR are same */

        rslt = bmi08a_set_power_mode(&bmi08dev);
        bmi08_error_codes_print_result("bmi08a_set_power_mode", rslt);

        rslt = bmi08xa_set_meas_conf(&bmi08dev);
        bmi08_error_codes_print_result("bmi08xa_set_meas_conf", rslt);

        bmi08dev.gyro_cfg.odr = BMI08_GYRO_BW_230_ODR_2000_HZ;
        bmi08dev.gyro_cfg.range = BMI08_GYRO_RANGE_250_DPS;
        bmi08dev.gyro_cfg.bw = BMI08_GYRO_BW_230_ODR_2000_HZ;
        bmi08dev.gyro_cfg.power = BMI08_GYRO_PM_NORMAL;

        rslt = bmi08g_set_power_mode(&bmi08dev);
        bmi08_error_codes_print_result("bmi08g_set_power_mode", rslt);

        rslt = bmi08g_set_meas_conf(&bmi08dev);
        bmi08_error_codes_print_result("bmi08g_set_meas_conf", rslt);
    }

    return rslt;

}

static int8_t enable_bmi08_interrupt()
{
    int8_t rslt;
    uint8_t data = 0;

    /* Set accel interrupt pin configuration */
    accel_int_config.int_channel = BMI08_INT_CHANNEL_1;
    accel_int_config.int_type = BMI08_ACCEL_INT_DATA_RDY;
    accel_int_config.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
    accel_int_config.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
    accel_int_config.int_pin_cfg.enable_int_pin = BMI08_ENABLE;

    /* Enable accel data ready interrupt channel */
    rslt = bmi08a_set_int_config((const struct bmi08_accel_int_channel_cfg*)&accel_int_config, &bmi08dev);
    bmi08_error_codes_print_result("bmi08a_set_int_config", rslt);

    if (rslt == BMI08_OK)
    {
        /* Set gyro interrupt pin configuration */
        gyro_int_config.int_channel = BMI08_INT_CHANNEL_3;
        gyro_int_config.int_type = BMI08_GYRO_INT_DATA_RDY;
        gyro_int_config.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
        gyro_int_config.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
        gyro_int_config.int_pin_cfg.enable_int_pin = BMI08_ENABLE;

        /* Enable gyro data ready interrupt channel */
        rslt = bmi08g_set_int_config((const struct bmi08_gyro_int_channel_cfg *)&gyro_int_config, &bmi08dev);
        bmi08_error_codes_print_result("bmi08g_set_int_config", rslt);

        rslt = bmi08g_get_regs(BMI08_REG_GYRO_INT3_INT4_IO_MAP, &data, 1, &bmi08dev);
        bmi08_error_codes_print_result("bmi08g_get_regs", rslt);
    }

    return rslt;
}


static int8_t disable_bmi08_interrupt()
{
    int8_t rslt;

    /* Set accel interrupt pin configuration */
    accel_int_config.int_channel = BMI08_INT_CHANNEL_1;
    accel_int_config.int_type = BMI08_ACCEL_INT_DATA_RDY;
    accel_int_config.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
    accel_int_config.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
    accel_int_config.int_pin_cfg.enable_int_pin = BMI08_DISABLE;

    /* Disable accel data ready interrupt channel */
    rslt = bmi08a_set_int_config((const struct bmi08_accel_int_channel_cfg*)&accel_int_config, &bmi08dev);
    bmi08_error_codes_print_result("bmi08a_set_int_config", rslt);

    if (rslt == BMI08_OK)
    {
        /* Set gyro interrupt pin configuration */
        gyro_int_config.int_channel = BMI08_INT_CHANNEL_3;
        gyro_int_config.int_type = BMI08_GYRO_INT_DATA_RDY;
        gyro_int_config.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
        gyro_int_config.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
        gyro_int_config.int_pin_cfg.enable_int_pin = BMI08_DISABLE;

        /* Disable gyro data ready interrupt channel */
        rslt = bmi08g_set_int_config((const struct bmi08_gyro_int_channel_cfg *)&gyro_int_config, &bmi08dev);
        bmi08_error_codes_print_result("bmi08g_set_int_config", rslt);
    }

    return rslt;
}

