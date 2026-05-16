/*
 * motor.c
 *
 *  Created on: 19 gru 2024
 *      Author: Szymon Nyderek
 */

#include "main.h"
#include "motor.h"
#include "LowPassFilter.h"
#include <string.h>

void Motor_CalculateSpeed(motor_t *motor)
{
	// (0.0) --  How many impulses did we get ?
	int32_t impulses;
	impulses = (int32_t)motor->EncoderValue - (int32_t)ROBOT_ENCODER_COUNTER_CENTER;
	impulses *= motor->EncoderDirection;
	motor->EncoderDeltaCounts = impulses;

	/*Distance traveled in 0.001s (One cycle)*/

	motor->DistanceInMeasurement = ((float)impulses * WHEEL_CIRCUMFERENCE) / ROBOT_ENCODER_COUNTS_PER_WHEEL_REV;
	/*Whole distance wheel has traveled*/
	motor->DistanceTraveled = motor->DistanceTraveled + motor->DistanceInMeasurement;

	/*Get meters per second*/
	motor->MetersPerSecond = motor->DistanceInMeasurement / ROBOT_CONTROL_DT_S;

	LowPassFilter_Update(&motor->MetersPerSecondLPF, motor->MetersPerSecond);

	motor->LpfDistanceInMeasurement = motor->MetersPerSecondLPF.output * ROBOT_CONTROL_DT_S;


    //How many times motor has rotated ?
	motor->NumberOfRotations = (float)impulses / ROBOT_ENCODER_COUNTS_PER_WHEEL_REV;
	//Rotations per minute based on period and impulses
	motor->RPM = motor->NumberOfRotations * (60.0f / ROBOT_CONTROL_DT_S);

	/*I know some of them are "magic values" other way it doesn't want to work*/

	if((motor->RPM >= 5000) || (motor->RPM <= -5000)) // |max| = 5000 Other readings are trash
	{
		motor->RPM = motor->PreviousRPM;
	}
	//Motor_SavePreviousRPMs(motor, motor->RPM);
	LowPassFilter_Update(&motor->EncoderRpmFilter, motor->RPM);
	motor->MetersPerHour = motor->MetersPerSecondLPF.output * 3600.0f;
	motor->KilometersPerHour = motor->MetersPerHour / 1000;
	motor->PreviousRPM = motor->EncoderRpmFilter.output;
}

void Motor_Init(motor_t *motor, float Kp, float Ki)
{
	memset(motor, 0, sizeof(*motor));
	motor->kp = Kp;
	motor->ki = Ki;
	motor->EncoderDirection = 1;
	motor->MetersPerSecondLPF.alpha = 0.1; // -- works like translation filter
}

void Motor_SetEncoderDirection(motor_t *motor, int8_t direction)
{
	motor->EncoderDirection = (direction < 0) ? -1 : 1;
}

void PI_Loop(motor_t *motor)
{
	motor->current_speed = motor->MetersPerSecondLPF.output * 23; // From m/s to bananas per second //65

	//Get absolute speed
	if(motor->current_speed < 0)
	{
		motor->current_speed = motor->current_speed * -1;
		//Zastanów się czy faktycznie to jest potrzebne
	}

	//Get the difference between speed that we need and actual speed
	motor->error =  motor->set_speed - motor->current_speed;

	//Add current error to previous errors
	motor->Error_sum = motor->Error_sum + motor->error;
	if(motor->Error_sum > 100)
	{
		motor->Error_sum = 100;
	}
	else if(motor->Error_sum < -100)
	{
		motor->Error_sum = -100;
	}

	//Calculate new speed with PI parameters
	motor->speed = motor->set_speed + (motor->error * motor->kp) + (motor->Error_sum *motor->ki);

}
