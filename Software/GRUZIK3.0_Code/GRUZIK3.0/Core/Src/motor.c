/*
 * motor.c
 *
 *  Created on: 19 gru 2024
 *      Author: Szymon Nyderek
 */

#include "main.h"
#include "motor.h"
#include "LowPassFilter.h"
static void Motor_SavePreviousRPMs(motor_t *motor, float rpm)
{
	float rpm_tmp[5];
	//If new RPM value is too big then it will be ignored
	rpm_tmp[4] = rpm;
	//Delete the oldest element and add new element
	for (int i =0; i <=3 ;i++)
	{
		rpm_tmp[i] = motor->PreviousRPMs[i+1];
	}
	for(int i =0; i <=4 ;i++)
	{
		motor->PreviousRPMs[i] = rpm_tmp[i];
	}
}
void Motor_CalculateSpeed(motor_t *motor)
{
	//How many impulses did we get ?
	int impulses;
	impulses = (int32_t)motor->EncoderValue - (int32_t)motor->EncoderPreviousValue;

	/*Distance traveled in 0.001s*/
	motor->DistanceInMeasurement = (impulses * WHEEL_CIRCUMFERENCE) / (IMPULSES_PER_ROTATION * GEAR_RATIO);
	/*Whole distance wheel has traveled*/
	motor->DistanceTraveled += motor->DistanceInMeasurement * -1;

//	/*Get meters per second*/
//	/* m/s = m/ms * 1000 */
//	motor->MetersPerSecond = motor->DistanceInMeasurement * -1000.0f; // 1s = 1000ms
//
//	LowPassFilter_Update(&motor->MetersPerSecondLPF, motor->MetersPerSecond);


    //How many times motor has rotated ?
	motor->NumberOfRotations = (float)impulses / 20000.0f;
	//Rotations per minute based on period and impulses
	motor->RPM = motor->NumberOfRotations * -60000.0f; //rotates per minute

	/*I know some of them are "magic values" other way it doesn't want to work*/

	if((motor->RPM >= 1000) || (motor->RPM <= -1000))
	{
		motor->RPM = motor->PreviousRPM;
	}
	//Motor_SavePreviousRPMs(motor, motor->RPM);
	LowPassFilter_Update(&motor->EncoderRpmFilter, motor->RPM);
	motor->MetersPerHour = motor->EncoderRpmFilter.output / RPM_TO_MH;
	//motor->MetersPerHour = motor->RPM / RPM_TO_MH;
	motor->KilometersPerHour = motor->MetersPerHour / 1000;
	motor->MetersPerSecond = motor->MetersPerHour / 3600;
	motor->PreviousRPM = motor->EncoderRpmFilter.output;
}

void Motor_Init(motor_t *motor, float Kp, float Ki)
{
	motor->kp = Kp;
	motor->ki = Ki;
}

void PI_Loop(motor_t *motor)
{
	//
	//TODO: Work on this with https://classroom.ubicoders.com/yt_simple_pid
	//
	//
	motor->current_speed = motor->MetersPerSecond * 65; // From m/s to bananas per second

	//Get absolute speed
	if(motor->current_speed < 0)
	{
		motor->current_speed = motor->current_speed * -1;
	}

	//Get the difference between speed that we need and actual speed
	motor->error =  motor->set_speed - motor->current_speed;

	//Add current error to previous errors
	motor->Error_sum = motor->Error_sum +motor->P;
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
