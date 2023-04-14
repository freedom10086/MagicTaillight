//
// Created by yang on 2023/4/9.
//

#include "bh1750.h"


static esp_err_t i2c_read_reg(uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, LIGHT_SENSOR_ADDR, &reg_addr,
                                        1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t i2c_read(uint8_t *data, size_t len) {
    return i2c_master_read_from_device(I2C_MASTER_NUM, LIGHT_SENSOR_ADDR,
                                       data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t i2c_write_byte(uint8_t reg_addr, uint8_t data) {
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, LIGHT_SENSOR_ADDR,
                                     write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}

static esp_err_t i2c_write_cmd(uint8_t reg_addr) {
    int ret;
    uint8_t write_buf[1] = {reg_addr};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, LIGHT_SENSOR_ADDR,
                                     write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}

void light_sensor_reset() {
    i2c_write_cmd(LIGHT_SENSOR_CMD_RESET);
}

void light_sensor_on() {
    i2c_write_cmd(LIGHT_SENSOR_CMD_ON);
}

void light_sensor_off() {
    i2c_write_cmd(LIGHT_SENSOR_CMD_OFF);
}

void light_sensor_start(int mode) {
    uint8_t cmd;
    switch (mode) {
        case CONTINUE_HRES_MODE:
            cmd = LIGHT_SENSOR_CMD_CONTINUE_HRES_MODE;
            break;
        case CONTINUE_HRES_MODE2:
            cmd = LIGHT_SENSOR_CMD_CONTINUE_HRES_MODE2;
            break;
        case CONTINUE_LRES_MODE:
            cmd = LIGHT_SENSOR_CMD_CONTINUE_LRES_MODE;
            break;
        case HRES_MODE:
            cmd = LIGHT_SENSOR_CMD_HRES_MODE;
            break;
        case HRES_MODE2:
            cmd = LIGHT_SENSOR_CMD_HRES_MODE2;
            break;
        case LRES_MODE:
            cmd = LIGHT_SENSOR_CMD_LRES_MODE;
            break;
        default:
            return;
    }

    i2c_write_cmd(cmd);
}

uint16_t light_sensor_read_light() {
    static uint8_t data[2];
    i2c_read(data, 2);
    uint16_t light = data[0] << 8 | data[1];
    return light;
}