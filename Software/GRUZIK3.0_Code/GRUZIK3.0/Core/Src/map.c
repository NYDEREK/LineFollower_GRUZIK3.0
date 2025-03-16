/*
 * map.c
 *
 *  Created on: Mar 5, 2025
 *      Author: SNYDE
 */

#include "main.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "map.h"
#include "Line_Follower.h"
#include "motor.h"
#include "math.h"
#include "app_fatfs.h"

// -- SD card -- //
extern FRESULT FatFsResult;
extern FATFS SdFatFs;
extern FIL SdCardFile;

void MapUpdate(Map_t *map, motor_t *MotorLeft, motor_t *MotorRight)
{
	if(map->Mapping == 1)
	{
		uint8_t buffer[100];
		//static int i;
		//(2.11) -- Translation In Measurement
		float Ti = (MotorLeft->LpfDistanceInMeasurement + MotorRight->LpfDistanceInMeasurement) / 2;
		//(2.12) -- Rotation In Measurement
		float Ri = (MotorRight->LpfDistanceInMeasurement - MotorLeft->LpfDistanceInMeasurement) / (MAIN_PCB_LENGTH * 2 * 1.05);

		//(2.13) -- Main PCB Position
		map->Xri = map->Xri + (Ti * cosf(map->Ori));
		map->Yri = map->Yri + (Ti * sinf(map->Ori));
		map->Ori = map->Ori + Ri;

		//(2.17) -- Sensor Position
		map->Pci[0] = map->Xri + (MAIN_TO_SENSOR_LENGTH * cosf(map->Ori));
		map->Pci[1] = map->Yri + (MAIN_TO_SENSOR_LENGTH * sinf(map->Ori));


		sprintf((char*)buffer, " %0.3f	%0.3f \n", map->Xri, map->Yri);
		f_printf(&SdCardFile, (char*)buffer);
	}
}
