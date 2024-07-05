/*
 * i2c_slave.c
 *
 *
 *  Created on: Jan 16, 2024
 *      Author: theheri24
 *
 *  Descripcion:
 *  		El MCU STM32 responde como un esclavo en la comunicacion I2C,
 *  		donde espera recibir los datos que registra los 6 sensores TOF del
 *  		Robot Robbie3
 *
 *  		-El Led status del robot alterna para indicar que recibe datos.
 *
 *  		-En este codigo se ejecuta en 2 secuencias,
 *
 *  			el primero regista la cantidad de bits que tiene
 *  			que recibir el MCU STM32, donde cada Bits es un caracter.
 *
 *  			el segundo recibe la trama completa de los datos, por la cantidad
 *  			de bits que se definio en el primero.
 *
 *  		En el archivo de main una funcion, Empiza a realizar la secuencia y en
 *  		en este archivo continua el ciclo.
 *
 */
#include "i2c_slave.h"

uint8_t received_length_i2c = 0;
char data_to_send[40];

// para la interrupcion
uint8_t flag_i2c = 0;
uint8_t flag_ready = 0;


unsigned int sensor_1 = 0;
unsigned int sensor_2 = 0;
unsigned int sensor_3 = 0;
unsigned int sensor_4 = 0;
unsigned int sensor_5 = 0;
unsigned int sensor_6 = 0;


void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if (flag_i2c == 1) {
		flag_i2c =0;

		sscanf(data_to_send, "%u %u %u %u %u %u", &sensor_1, &sensor_2, &sensor_3, &sensor_4, &sensor_5, &sensor_6);
//		formatea la cadena que registra los datos de los sensores TOF
		memset(data_to_send, 0, sizeof(data_to_send));
//		lee la cantidad de caracteres que espera recibir.
		HAL_I2C_Slave_Receive_DMA(&hi2c2,&received_length_i2c, 1);
	} else {
		flag_i2c =1;

//		HAL_GPIO_TogglePin(STATUS_LED_RED_GPIO_Port, STATUS_LED_RED_Pin);
//		registra la cantidad de caracteres por el tamaño definido en received_length_i2c
		HAL_I2C_Slave_Receive_DMA(&hi2c2,(uint8_t*)data_to_send,received_length_i2c);
	}
}
