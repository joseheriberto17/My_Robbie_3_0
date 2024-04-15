/*
 * control_motor.h
 *
 *  Created on: Oct 27, 2023
 *      Author: theheri24
 */

#ifndef INC_CONTROL_MOTOR_H_
#define INC_CONTROL_MOTOR_H_
#include "main.h"



#include <stdlib.h>
#include <math.h>

extern int32_t cnt1_1;
extern int32_t cnt1_2;
extern int32_t cnt2_1;
extern int32_t cnt2_2;

extern float q0;
extern float q1;
extern float q2;





int pulse_To_Sample_Time(TIM_HandleTypeDef *htim,int32_t cnt_2, int32_t cnt_1);
float Convert_Pulse_To_Rpm(int32_t counter, int32_t sample_time);


#endif /* INC_CONTROL_MOTOR_H_ */
