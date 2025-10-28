/*
 * motor.h
 *
 *  Created on: 19 gru 2024
 *      Author: Szymon Nyderek
 */
#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"
#include "LowPassFilter.h"

#define RPM_TO_MH 0.20833333333f
#define IMPULSES_PER_ROTATION 2000.0f
#define GEAR_RATIO 10.0f
#define MEASUREMENT_PERIOD 0.001f // 0.001s 1KHZ
#define WHEEL_CIRCUMFERENCE 0.0706858347f//0.0706858347f //0.08f


typedef struct
{
	/*Encoder and speed*/
	uint32_t EncoderValue;
	uint32_t EncoderPreviousValue;  //was on uint16_t
	float RPM;
	float PreviousRPM;
	float PreviousRPMs[5];
	float NumberOfRotations;
	float MetersPerHour;
	float KilometersPerHour;
	float MetersPerSecond;

	LowPassFilter_t EncoderRpmFilter;
	LowPassFilter_t MetersPerSecondLPF;

	/*Tracking*/
	float DistanceInMeasurement;
	float LpfDistanceInMeasurement;
	float DistanceTraveled;


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
void Motor_Init(motor_t *motor, float Kp, float Ki);
void Motor_CalculateSpeed(motor_t *motor);
void PI_Loop(motor_t *motor);


#endif /* INC_MOTOR_H_ */
