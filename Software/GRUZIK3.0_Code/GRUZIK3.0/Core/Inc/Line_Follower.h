/*
 * Line_Follower.h
 *
 *  Created on: 18 paź 2024
 *      Author: Szymon
 */

#ifndef INC_LINE_FOLLOWER_H_
#define INC_LINE_FOLLOWER_H_

#include "main.h"

typedef struct
{
	uint8_t PowerMode;
	float Kp;
	float Kd;
	float Error_P;
	float Error_D;

	int Base_speed_R;
	int Base_speed_L;
	int Max_speed_R;
	int Max_speed_L;

	int Sharp_bend_speed_right;
	int Sharp_bend_speed_left;
	int Bend_speed_right;
	int Bend_speed_left;

	float battery_voltage;
	uint16_t Adc_Value;
	float Speed_level;
	float Speed_offset;

	float MaxSpeed;
	float AverageSpeed;
	float AverageSpeedSum;
	float AverageSpeedNum;

	uint8_t DrivingOnMap;
	uint8_t blockinterrups;
	uint8_t LineFollowing;

	uint8_t SensorValues[12];
	uint16_t treshold;
	uint32_t LastEndTimer;
	int Last_end;
	int Last_idle;
	int SensorPosition;
	int actives;

	float P;
	float D;
	int Errors[10];
	float Last_error;

	float Yaw;
	float Orientation;

	uint32_t DoneUnMappingTimer;
	uint8_t UnMappingDone;

	uint8_t state;

} LineFollower_t;
enum state
{
	PidFollowing = 0,
	Mapping = 1,
	UnMapping = 2,
	UnSpeedMapping = 3,
};

enum PowerState
{
	Initialization = 0,
	Stop = 1,
	Start = 2,
};

#endif /* INC_LINE_FOLLOWER_H_ */
