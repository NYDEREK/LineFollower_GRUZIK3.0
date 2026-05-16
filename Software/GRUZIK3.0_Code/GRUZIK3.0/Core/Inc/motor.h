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
#include "robot_config.h"

#define RPM_TO_MH 0.20833333333f
#define IMPULSES_PER_ROTATION ROBOT_ENCODER_STEPS_PER_MOTOR_REV
#define GEAR_RATIO ROBOT_GEAR_RATIO
#define MEASUREMENT_PERIOD ROBOT_CONTROL_DT_S
#define WHEEL_CIRCUMFERENCE ROBOT_WHEEL_CIRCUMFERENCE_M


typedef struct
{
	/*Encoder and speed*/
	uint32_t EncoderValue;
	int32_t EncoderDeltaCounts;
	int8_t EncoderDirection;
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
void Motor_SetEncoderDirection(motor_t *motor, int8_t direction);
void Motor_CalculateSpeed(motor_t *motor);
void PI_Loop(motor_t *motor);


#endif /* INC_MOTOR_H_ */
