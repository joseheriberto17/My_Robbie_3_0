/*
 * i2c_slave.c
 *
 *
 *  Created on: Jan 16, 2024
 *      Author: theheri24
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


/*
 *  * parametro para uso:
 *      -pin 14 y 15 sda y scl en el stm32 tipo arduino
 *		-hi2c1 modo polling, sin interrupcion
 *		-dma con interrupcion en el huart2
 *
 *	entrada;
 *	 un dispositiv conectado al stm32 manda primero un valor, la cual es el
 *	 numero de bytes que va a leer, luego se prepara para leer todo el tramo
 *	 siguiente.
 *
 *
 */

//activar para modo DMA

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if (flag_i2c == 1) {
		flag_i2c =0;

//		sprintf(MSG_Tx,"size: %d datos: %s \r\n",received_length,data_to_send);
//		memset(data_to_send, 0, sizeof(data_to_send));
		sscanf(data_to_send, "%u %u %u %u %u %u", &sensor_1, &sensor_2, &sensor_3, &sensor_4, &sensor_5, &sensor_6);
		memset(data_to_send, 0, sizeof(data_to_send));

		HAL_I2C_Slave_Receive_DMA(&hi2c2,&received_length_i2c, 1);
	} else {
		flag_i2c =1;

		//la funcion que ayuda a que no se muestre el caracter interpreado como \0 de valor i2c enviado.
//		memset(data_to_send, 0, sizeof(data_to_send));
//		memset(data_Tx, 0, sizeof(data_Tx));

		HAL_GPIO_TogglePin(STATUS_LED_RED_GPIO_Port, STATUS_LED_RED_Pin);

		HAL_I2C_Slave_Receive_DMA(&hi2c2,(uint8_t*)data_to_send,received_length_i2c);
	}
}
