/*
 * map.h
 *
 *  Created on: Mar 5, 2025
 *      Author: SNYDE
 */

#ifndef INC_MAP_H_
#define INC_MAP_H_

#include "main.h"
#include "Line_Follower.h"
#include "motor.h"
#include "odometry.h"
#include "robot_config.h"

#define MAIN_PCB_LENGTH ROBOT_WHEEL_BASE_M
#define MAIN_TO_SENSOR_LENGTH ROBOT_SENSOR_OFFSET_M
#define MAP_SIZE 3000


typedef struct {
	uint8_t Mapping;
	uint8_t PlaybackActive;
	float Map[MAP_SIZE][2];
	///
	float Kk; // -- curvature to speed adjuster

	///
	float WayPoint[2][5];
	float WayPointError;
	float WayPointSpeedCompensation;
	////
	float Ti;
	float Ri;
	///
	float Xri, PreviousXri;
	float Yri, PreviousYri;
	float Ori, PreviousOri, OriIMU;
	float AlhphaOri;
	///
	float Qri[3]; //(Xri, Yri, Ori)
	float Pci[2], PreviousPci[2];
	///
	float Si;
	float Ai , PreviousAi;
	float Ki;

	/// - driving on map
	float SetX;
	float SetY;
	float PreviousSetX;
	float PreviousSetY;
	float SetRotation;
	float SetSpeed;
	float DefaultPlaybackSpeed;

	//PID
	float p;
	float i;
	float d;
	float ErrorSum;
	float LastError;
	float FilteredErrorDerivative;
	float mi;

	float StartXri;
	float StartYri;
	float StartOri;
	float StartPci[2];
	uint16_t RecordedPoints;
	uint8_t loopNumber;
	uint8_t TargetValid;
	uint8_t StartValid;
	uint8_t AutoClosed;

} Map_t;

//Methods
void Map_Reset(Map_t *map);
uint8_t Map_BeginPlayback(Map_t *map);
void Map_SyncPose(Map_t *map, const Pose2D_t *pose, float delta_distance_m);
uint8_t MapUpdate(Map_t *map, const Pose2D_t *pose, float speed_mps, float error_p, float error_d);
uint8_t DriveOnMap(Map_t *map, const Pose2D_t *pose);
uint8_t MapUpdateV2(Map_t *map, const Pose2D_t *pose, float speed_mps, float error_p, float error_d);
uint8_t SpeedUnMapping(Map_t *map, const Pose2D_t *pose);
#endif /* INC_MAP_H_ */
