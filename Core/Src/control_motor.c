/*
 * control_motor.c
 *
 *  Created on: Oct 27, 2023
 *      Author: theheri24
 *
 *		-Este archivo es una extension del archivo main.
 *
 *		-Posee un algoritmo que actua como un controlador PID, que permite controlar
 *		 la velocidad de las  2 ruedas de robot Robbie3.
 *
 *		-Registra y organiza las variable que nesecita el robot publicar para la
 *		 aplicacion propuesta, esto lo hace para los 2 perifricos UART que estan conectado
 *		 en el robot.
 *
 *		-El MCU STM32 usa el Timer5 para ejecutar el codigo cada 100 ms.
 */
#include <control_motor.h>

// variable

//encoders
int32_t cnt1_1 = 0;
int32_t cnt1_2 = 0;
int32_t cnt2_1 = 0;
int32_t cnt2_2 = 0;

//PWM
int16_t Duty_1 = 0;
int16_t Duty_2 = 0;

// tiempo de muestreo estimado (ms)
uint8_t sample_time = 100;
uint32_t counter_time = 0;
int32_t diff_time = 0;

//base tiempo
uint16_t time_1;
uint16_t time_2;

//constantes PID
float q0=1.32;
float q1=-0.45;
float q2=0;

//control motor 1
float uk1 = 0.000;
float uk1_1 = 0.000;
float uk1_2 = 0.000;

float ek1 = 0.000;
float ek1_1 = 0.000;
float ek1_2 = 0.000;

//control motor 1
float uk2 = 0.000;
float uk2_1 = 0.000;
float uk2_2 = 0.000;

float ek2 = 0.000;
float ek2_1 = 0.000;
float ek2_2 = 0.000;
/**
  * @brief  esta fucion determina si los dos reciente contadores del
  * 	    encoder pasan por el desbordamiento del timer y
  * 	    en que direccion (asiende o desient)
  * @param  cnt_2 = valor del contador del encoder en el tiempo 2.
  * @param  cnt_1 = valor del contador del encoder en el tiempo 1.
  * @retval
 */
int pulse_To_Sample_Time(TIM_HandleTypeDef *htim,int32_t cnt_2, int32_t cnt_1) {
	//orientacion del giro de motor
	int valor = 0;
	if (abs(cnt_2 - cnt_1) > 200) {
		if (cnt_1 > cnt_2) {
			valor = 1;
		} else {
			valor = -1;
		}
	} else {
		valor = 0;
	}
	return valor*(__HAL_TIM_GET_AUTORELOAD(htim)+1) + cnt_2 - cnt_1;
}
/**
  * @brief  conversion de pulso por segundo a RPM.
  * @param  valor de la diferencia del contador a un tiempo sample_time
  * @param  tiempo que estimo que colapsa el timer , periodo de muestreo definido
  * @retval varible pulso por segundo.
 */
float Convert_Pulse_To_Rpm(int32_t counter, int32_t sample_time){
	int PPR = 7; // pulso por vuelta de rotor
	int RR = 100; // relacion rotor y eje del motor

	float value = (((((float)counter/sample_time)*1000)/PPR)*60)/RR;
	return value;
}
/*
 * cada vez que colapsa el counter de tiempo llega un un tiempo fijo para procesar el
 * controlador
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim->Instance == TIM5){

//		XXXX_1 = datos de motor izquierdo
//		XXXX_2 = datos de motor derecho

		cnt1_2 = __HAL_TIM_GET_COUNTER(&htim1);
		cnt2_2 = __HAL_TIM_GET_COUNTER(&htim2);

		int32_t diff_1 = 0;
		int32_t diff_2 = 0;
		float speed_1 = 0.000;
		float speed_2 = 0.000;

		counter_time++;

//		lineas para la prueba movilidad en el robot
//		secuencia(counter_time);
//		setP_1 = 45;
//		setP_2 = 45;

		diff_1 = pulse_To_Sample_Time(&htim1,cnt1_2, cnt1_1);
		diff_2 = pulse_To_Sample_Time(&htim2,cnt2_2, cnt2_1);

		// convertir a porcentaje de RPM
		speed_1 = 100.000*(Convert_Pulse_To_Rpm(diff_1, sample_time)/114.000);
		speed_2 = 100.000*(Convert_Pulse_To_Rpm(diff_2, sample_time)/114.000);

//		// velocidad sin controlador
//		uk1 = setP_1;
//		uk2 = -setP_2;

		//error motores
		ek1 = setP_1-speed_1;
		ek2 = -setP_2-speed_2;

//		accion de control
		if(setP_1 == 0){
			uk1 = 0;
		}else{
			uk1 =q0*ek1 + q1*ek1_1 + q2*ek1_2 + uk1_1;

		}

		if(setP_2 == 0){
			uk2 = 0;
		}else{
			uk2 =q0*ek2 + q1*ek2_1 + q2*ek2_2 + uk2_1;
		}

		//limitador de accion de control
		if (uk1>=100.0){
			uk1=100.0;
		}
		if (uk1<=-100.0){
			uk1=-100.0;
		}

		if (uk2>=100.0){
			uk2=100.0;
		}
		if (uk2<=-100.0){
			uk2=-100.0;
		}



		Duty_1 = abs(uk1*(999/100.0));
		if(uk1>=0){
			//dir motor de izquierda
			HAL_GPIO_WritePin(AMOT2_GPIO_Port, AMOT2_Pin, GPIO_PIN_RESET);
			//dir motor izquierdo
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,Duty_1);

		} else {
			//dir motor de izquierda
			HAL_GPIO_WritePin(AMOT2_GPIO_Port, AMOT2_Pin, GPIO_PIN_SET);
			//dir motor izquierdo
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,__HAL_TIM_GET_AUTORELOAD(&htim3)+1-Duty_1);

		}

		Duty_2 = abs(uk2*(999/100.0));
		if(uk2<=0){
			//dir motor de derecha
			HAL_GPIO_WritePin(BMOT2_GPIO_Port, BMOT2_Pin, GPIO_PIN_RESET);
			//dir motor derecho
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,Duty_2);
		} else {
			//dir motor de derecha
			HAL_GPIO_WritePin(BMOT2_GPIO_Port, BMOT2_Pin, GPIO_PIN_SET);
			//dir motor derecho
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,__HAL_TIM_GET_AUTORELOAD(&htim3)+1-Duty_2);
		}

		/*
		 * publicacion del mensaje que registra los datos internos del robot.
		 * 		setP_x = velocidad esperada en % de la velocidad del robot.
		 * 		speed_x = velocidad estimado en % de la velocidad del robot
		 * 		sensor_x = valor distancia  que registra un sensor.
		 * 		counter_time = tiempo en segundos que transcurre en la ejecucion del firmware.
		 *
		 * 		XXXX_1 = datos de motor izquierdo
		 * 		XXXX_2 = datos de motor derecho
		 */
		sprintf(MSG_Tx,"A1: %03d,A2: %03d,"
					   "B1: %03d,B2: %03d,"
					   "C1: %03u,C2: %03u,C3: %03u,C4: %03u,C5: %03u,C6: %03u,"
					   "D1: %05u s,"
					   "E1: %s, E2: %u\r\n",
					   (int)setP_1,(int)setP_2,
					   (int)speed_1,(int)-speed_2,
					   sensor_1,sensor_2,sensor_3,sensor_4,sensor_5,sensor_6,
					   (int)counter_time/10,
					   MSG_Tx_1,strlen(MSG_Tx_1));

		// mensaje de respuesta se publica cada 100 ms al UART que se conecta al CH340 para la comunicacion por cable.
		HAL_UART_Transmit_DMA(&huart3, (uint8_t*)&MSG_Tx,strlen(MSG_Tx));

		// mensaje de respuesta se publica  cada 100 ms al UART que se conecta al ESP-12F.
		while ((HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX) ||
						(HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX_RX));
		HAL_UART_Transmit_IT(&huart1, (uint8_t*)&MSG_Tx,strlen(MSG_Tx));


		//shit register motor 1
		uk1_2=uk1_1;
		uk1_1=uk1;

		ek1_2=ek1_1;
		ek1_1=ek1;

		//shit register motor 2
		uk2_2=uk2_1;
		uk2_1=uk2;

		ek2_2=ek2_1;
		ek2_1=ek2;

		cnt1_1 = cnt1_2;
		cnt2_1 = cnt2_2;

		diff_time = __HAL_TIM_GET_COUNTER(&htim5);
	}
}

