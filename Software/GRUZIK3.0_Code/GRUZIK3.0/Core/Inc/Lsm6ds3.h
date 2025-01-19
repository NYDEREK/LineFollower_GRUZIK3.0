/*
 * Lsm6ds3.h
 *
 *  Created on: Dec 24, 2024
 *      Author: Szymon
 */
#include "main.h"
#include "LowPassFilter.h"

/*IIR filter*/
#define LSM6DS3_ACCEL_LPF_ALPHA 0.1
#define LSM6DS3_GYRO_LPF_ALPHA 0.01

#ifndef INC_LSM6DS3_H_
#define INC_LSM6DS3_H_

#define LSM6DS3_TIMEOUT			   1000

#define LSM6DS3_ADDRESS            0x6A<<1

#define LSM6DS3_WHO_AM_I_REG       0X0F
#define LSM6DS3_CTRL1_XL           0X10
#define LSM6DS3_CTRL2_G            0X11
#define LSM6DS3_CTRL9_XL           0X18

#define LSM6DS3_STATUS_REG         0X1E

#define LSM6DS3_CTRL6_C            0X15
#define LSM6DS3_CTRL7_G            0X16
#define LSM6DS3_CTRL8_XL           0X17

#define LSM6DS3_OUT_TEMP_L         0X20
/*Gyroscope*/
#define LSM6DS3_OUTX_L_G           0X22
#define LSM6DS3_OUTX_H_G           0X23

#define LSM6DS3_OUTY_L_G           0X24
#define LSM6DS3_OUTY_H_G           0X25

#define LSM6DS3_OUTZ_L_G           0X26
#define LSM6DS3_OUTZ_H_G           0X27

#define LSM6DS3_INT_CTRL		   0X0D
#define LSM6DS3_CTRL6_C            0X15
#define LSM6DS3_CTRL3_C            0X12
#define LSM6DS3_CTRL4_C            0X13
/*Accelerometer*/
#define LSM6DS3_OUTX_L_XL          0X28
#define LSM6DS3_OUTX_H_XL          0X29

#define LSM6DS3_OUTY_L_XL          0X2A
#define LSM6DS3_OUTY_H_XL          0X2B

#define LSM6DS3_OUTZ_L_XL          0X2C
#define LSM6DS3_OUTZ_H_XL          0X2D



typedef struct {

	 I2C_HandleTypeDef *I2C;
	/*Gyroscope*/
	float RawGx, RawGy, RawGz;
	/*Filtered gyroscope data*/
	LowPassFilter_t LpfGx,LpfGy,LpfGz;
	/*Filtered gyroscope in rad/s */
	float Gx, Gy, Gz;

	/*Accelerometer*/
	float RawAx, RawAy, RawAz;
	/*Filtered accelerometer data*/
	LowPassFilter_t LpfAx,LpfAy,LpfAz;
	/*Filtered accelerations in m/s2*/
	float Ax, Ay, Az;


	/*Temperature*/
	float Temperature;

} Lsm6ds3_t;

uint8_t LSM_Init(Lsm6ds3_t *lsm, I2C_HandleTypeDef *i2c);
void LSM_readGyroscope(Lsm6ds3_t *lsm);
void LSM_readAcceleration(Lsm6ds3_t *lsm);
void LSM_readTemperature(Lsm6ds3_t *lsm);

#endif /* INC_LSM6DS3_H_ */
