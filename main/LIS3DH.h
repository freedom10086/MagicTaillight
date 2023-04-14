//
// Created by yang on 2023/4/9.
//

#ifndef BLINK_LIS3DH_H
#define BLINK_LIS3DH_H

#include "driver/i2c.h"


// 0011001 sa0 = 1 , default pull up
// 0011000 sa0 = 0
#define LIS3DH_ADDR 0B0011001

#define LIS3DH_REG_STATUS 0x07


uint8_t lis3dh_read_status();

#endif //BLINK_LIS3DH_H
