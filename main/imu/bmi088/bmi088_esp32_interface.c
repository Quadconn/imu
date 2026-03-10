#include "bmi088_esp32_interface.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "bmi08_defs.h"

#include "driver/i2c_master.h"
#include "esp_rom_sys.h"


#define WAIT_FOREVER -1
#define TX_BUFF_LEN_MAX 32

static i2c_master_bus_config_t i2c_mst_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port   = I2C_NUM_0,
    .scl_io_num = GPIO_NUM_2,
    .sda_io_num = GPIO_NUM_1,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = false,
};

static i2c_master_bus_handle_t bus_handle;

// NOTE: Max i2c clock speed is 400kHz for esp32s3 and bmi088

static i2c_device_config_t dev_cfg_gyro = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address  = BMI08_GYRO_I2C_ADDR_SECONDARY,
    .scl_speed_hz    = 100E3,
};

i2c_master_dev_handle_t dev_handle_gyro;


static i2c_device_config_t dev_cfg_accel = { .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address  = BMI08_ACCEL_I2C_ADDR_SECONDARY,
    .scl_speed_hz    = 100E3,
};

i2c_master_dev_handle_t dev_handle_accel;


static BMI08_INTF_RET_TYPE bmi088_i2c_read(uint8_t reg_addr, uint8_t *reg_data, 
                                    uint32_t len, void *intf_ptr);

static BMI08_INTF_RET_TYPE bmi088_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, 
                                     uint32_t len, void *intf_ptr);

static void bmi088_delay_us(uint32_t period, void *intf_ptr);


bool bmi088_esp32_interface_init(struct bmi08_dev* bmi088) {
    esp_err_t err;
    // I2C Init
    err = i2c_new_master_bus(&i2c_mst_config, &bus_handle);
    if (err != ESP_OK) {
        return false;
    }

    err = i2c_master_bus_add_device(bus_handle, &dev_cfg_gyro, &dev_handle_gyro);
    if (err != ESP_OK) {
        return false;
    }

    err = i2c_master_bus_add_device(bus_handle, &dev_cfg_accel, &dev_handle_accel);
    if (err != ESP_OK) {
        return false;
    }

    // BMI088 Init
    bmi088->intf = BMI08_I2C_INTF;
    bmi088->read = bmi088_i2c_read;
    bmi088->write = bmi088_i2c_write;
    bmi088->variant = BMI088_VARIANT;
    bmi088->intf_ptr_accel = (void*)dev_handle_accel;
    bmi088->intf_ptr_gyro = (void*)dev_handle_gyro;
    bmi088->delay_us = bmi088_delay_us;
    bmi088->read_write_len = TX_BUFF_LEN_MAX;

    return true;
}

// NOTE: Using intf_ptr to select proper i2c device

static BMI08_INTF_RET_TYPE bmi088_i2c_read(uint8_t reg_addr, uint8_t *reg_data, 
                                    uint32_t len, void *intf_ptr) {
    esp_err_t err;

    i2c_master_dev_handle_t dev_handle = (i2c_master_dev_handle_t)intf_ptr;

    err = i2c_master_transmit_receive(dev_handle, &reg_addr, sizeof(uint8_t), reg_data, len, WAIT_FOREVER);

    if (err != ESP_OK) {
        return !BMI08_INTF_RET_SUCCESS;
    }

    return BMI08_INTF_RET_SUCCESS;
}


static BMI08_INTF_RET_TYPE bmi088_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, 
                                     uint32_t len, void *intf_ptr) {

    esp_err_t err;
    static uint8_t tx_buff[TX_BUFF_LEN_MAX * 2] = {0};

    i2c_master_dev_handle_t dev_handle = (i2c_master_dev_handle_t)intf_ptr;

    if (len > TX_BUFF_LEN_MAX) {
        return !BMI08_INTF_RET_SUCCESS;
    }

    tx_buff[0] = reg_addr;
    memcpy(&tx_buff[1], reg_data, len);

    err = i2c_master_transmit(dev_handle, tx_buff, len + 1, WAIT_FOREVER);

    if (err != ESP_OK) {
        return !BMI08_INTF_RET_SUCCESS;
    }

    return BMI08_INTF_RET_SUCCESS;
}


static void bmi088_delay_us(uint32_t period, void *intf_ptr) {
    // NOTE: May have to upgrade to actual timer if this is not good enough during
    // testing.
    esp_rom_delay_us(period);
}

