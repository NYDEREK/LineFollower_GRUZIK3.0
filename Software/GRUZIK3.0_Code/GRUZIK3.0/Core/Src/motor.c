/*
 * motor.c
 *
 *  Created on: 19 gru 2024
 *      Author: Szymon
 */

#include "main.h"
#include "motor.h"

void Motor_CalculateSpeed(motor_t *motor)
{
	//TODO: daj tu normalne pewne wartosci pomierz wszytsko jak nalezy zeby było ładnie i wstaw zamiast 0.001 numbersofrotations itd
    //How many impulses did we get ?
	int impulses = motor->EncoderValue - motor->EncoderPreviousValue;
    //How many times motor has rotated ?
	if(motor->ImpulsesPerRotation != 0)
	{
		motor->NumberOfRotations = impulses * 0.001;
	}
	//Rotations per minute based on period
	if(motor->period != 0)
	{
		motor->RPM = motor->NumberOfRotations / 0.01; //rotates per 10ms
	}

	motor->KilometersPerHour = motor->RPM * 0.0048;
}

void Motor_Init(motor_t *motor, float Kp, float Ki, uint16_t ImpulsesPerRotation, float period)
{
	motor->kp = Kp;
	motor->ki = Ki;
	motor->ImpulsesPerRotation = ImpulsesPerRotation;
	motor->period = period;
}

void PI_Loop(motor_t *motor)
{
	motor->P =  motor->set_speed - motor->current_speed;

	for (int i = 0; i < 5; i++)
	{
		motor->Error_sum +=  motor->Errors[i];
	}

	motor->I = motor->Error_sum;

	motor->speed = (motor->P * motor->kp) + (motor->I *motor->ki);
}