//
// Created by yang on 2023/4/9.
//

#include "LIS3DH.h"

#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_TIMEOUT_MS       1000

static esp_err_t i2c_read_reg(uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, LIS3DH_ADDR, &reg_addr,
                                        1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t i2c_read(uint8_t *data, size_t len) {
    return i2c_master_read_from_device(I2C_MASTER_NUM, LIS3DH_ADDR,
                                       data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t i2c_write_byte(uint8_t reg_addr, uint8_t data) {
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, LIS3DH_ADDR,
                                     write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}

static esp_err_t i2c_write_cmd(uint8_t reg_addr) {
    int ret;
    uint8_t write_buf[1] = {reg_addr};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, LIS3DH_ADDR,
                                     write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}

uint8_t lis3dh_read_status() {
    uint8_t d[1] = {0};
    i2c_read_reg(LIS3DH_REG_STATUS, d, 1);
    return d[0];
}