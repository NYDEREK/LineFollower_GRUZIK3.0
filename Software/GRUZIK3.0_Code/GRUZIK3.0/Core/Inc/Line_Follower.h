/*
 * Line_Follower.h
 *
 *  Created on: 18 paź 2024
 *      Author: Szymon
 */

#ifndef INC_LINE_FOLLOWER_H_
#define INC_LINE_FOLLOWER_H_

typedef struct
{
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

	uint8_t DrivingOnMap;
	uint8_t blockinterrups;

	uint32_t DoneUnMappingTimer;
	uint8_t UnMappingDone;

	uint8_t state;

} LineFollower_t;
enum state
{
	PidFollowing,
	Mappig,
	UnMapping,
	Stop,
};

#endif /* INC_LINE_FOLLOWER_H_ */
