/*
 * motor.h
 *
 *  Created on: 19 gru 2024
 *      Author: Szymon
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"
typedef struct
{
	/*Encoder and speed*/
	uint16_t EncoderValue;
	uint16_t EncoderPreviousValue;
	float RPM;
	uint16_t ImpulsesPerRotation; //10000
	float period; // 0.01   ---> 0.01s -> 10ms
	float NumberOfRotations;
	float KilometersPerHour;

	/*PI algorithm*/
	float set_speed;
	float current_speed;
	float speed;

	float error;
	float Errors[10];
	float Error_sum;

	float ki;
	float kp;
	float P;
	float I;

} motor_t;

/*functions*/
void Motor_Init(motor_t *motor, float Kp, float Ki, uint16_t ImpulsesPerRotation, float period);
void Motor_CalculateSpeed(motor_t *motor);
void PI_Loop(motor_t *motor);


#endif /* INC_MOTOR_H_ */