/*
 * i2c_slave.h
 *
 *  Created on: Jan 16, 2024
 *      Author: theheri24
 */

#ifndef INC_I2C_SLAVE_H_
#define INC_I2C_SLAVE_H_


#include "string.h"
#include <stdio.h>
#include "main.h"
#include "stm32f4xx_hal.h"

extern char data_to_send[40];

extern unsigned int sensor_1;
extern unsigned int sensor_2;
extern unsigned int sensor_3;
extern unsigned int sensor_4;
extern unsigned int sensor_5;
extern unsigned int sensor_6;

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c);

#endif /* INC_I2C_SLAVE_H_ */
