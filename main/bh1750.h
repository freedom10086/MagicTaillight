//
// Created by yang on 2023/4/9.
//

#ifndef BLINK_BH1750_H
#define BLINK_BH1750_H

#include "driver/i2c.h"


#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_TIMEOUT_MS       1000

// ADDR lOW 0100011
// ADDR high 1011100
#define LIGHT_SENSOR_ADDR 0x23

#define LIGHT_SENSOR_CMD_OFF 0x00
#define LIGHT_SENSOR_CMD_ON 0x01

#define LIGHT_SENSOR_CMD_RESET 0B00000111
// 1lx resolution 120ms
#define LIGHT_SENSOR_CMD_CONTINUE_HRES_MODE 0x10
// 0.5lx resolution  120ms
#define LIGHT_SENSOR_CMD_CONTINUE_HRES_MODE2 0B00010001
// 4lx resolution 16ms.
#define LIGHT_SENSOR_CMD_CONTINUE_LRES_MODE 0B00010011
//  1lx resolution.
#define LIGHT_SENSOR_CMD_HRES_MODE 0B00100000
// 0.5lx resolution  120ms
#define LIGHT_SENSOR_CMD_HRES_MODE2 0B00100001
// 4lx resolution 16ms.
#define LIGHT_SENSOR_CMD_LRES_MODE 0B00100011

enum bh1750_mode {
    CONTINUE_HRES_MODE,
    CONTINUE_HRES_MODE2,
    CONTINUE_LRES_MODE,
    HRES_MODE,
    HRES_MODE2,
    LRES_MODE
};

//Change Measurement time
// High bit  01000_MT[7,6,5]
// Low bit 011_MT[4,3,2,1,0]

void light_sensor_reset();

void light_sensor_on();

void light_sensor_off();

void light_sensor_start(int mode);

uint16_t light_sensor_read_light();

#endif //BLINK_BH1750_H
