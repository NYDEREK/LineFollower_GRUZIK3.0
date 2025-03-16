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

#define MAIN_PCB_LENGTH 0.155f
#define MAIN_TO_SENSOR_LENGTH 0.160f //TODO: zmierz to normalnie albo zoba w programie
#define MAP_SIZE 3000


typedef struct {
	uint8_t Mapping;
	float Map[MAP_SIZE][2];

	////
	float Ti;
	float Ri;
	///
	float Xri, PreviousXri;
	float Yri, PreviousYri;
	float Ori, PreviousOri;
	///
	float Qri[3]; //(Xri, Yri, Ori)
	float Pci[2];

} Map_t;

//Methods
void MapUpdate(Map_t *map, motor_t *MotorLeft, motor_t *MotorRight);

#endif /* INC_MAP_H_ */
