/*
 * Lsm6ds3.c
 *
 *  Created on: Dec 24, 2024
 *      Author: Szymon
 */
#include "main.h"
#include "Lsm6ds3.h"
#include "i2c.h"
#include "string.h"
#include "stdio.h"
#include "LowPassFilter.h"

/*I2C Functions for communication*/
uint8_t readRegister(Lsm6ds3_t *lsm, uint8_t address)
{
	uint8_t value;

	HAL_I2C_Mem_Read(lsm->I2C, ((0x6A)<<1), address, 1, &value, 1, LSM6DS3_TIMEOUT);

	return value;
}
void writeRegister(Lsm6ds3_t *lsm, uint8_t address, uint8_t value)
{
	HAL_I2C_Mem_Write(lsm->I2C, ((0x6A)<<1), address, 1, &value, 1, LSM6DS3_TIMEOUT);
}
void readRegisters(Lsm6ds3_t *lsm, uint8_t address, uint8_t *data, size_t length)
{
	uint8_t values[length];

	HAL_I2C_Mem_Read(lsm->I2C, ((0x6A)<<1), address, 1, values, sizeof(values), LSM6DS3_TIMEOUT);
}
/*Library Functions*/
uint8_t LSM_Init(Lsm6ds3_t *lsm, I2C_HandleTypeDef *i2c)
{
	  uint8_t LsmID;
	  /*Set the I2C to chosen one*/
	  lsm->I2C = i2c;
	  /*Check if we have good conection with Lsm6ds3*/
	  LsmID = readRegister(lsm, LSM6DS3_WHO_AM_I_REG);

	  if(LsmID !=  0x69)
	  {
		  return 1;
	  }
	  /*Gyroscope Initialization*/

	  //set the gyroscope control register to work at 1.66 KHz, 2000 DPS
	  writeRegister(lsm, LSM6DS3_CTRL2_G, 0x8C); //1000 1100 - 2000dps

	  //Set the LPF1 to 224Hz
	  writeRegister(lsm, LSM6DS3_CTRL6_C, 0x01);// 0000 0001

	  // set gyroscope power mode to high performance and bandwidth to 16 MHz
	  writeRegister(lsm, LSM6DS3_CTRL7_G, 0x00);



	  // Set the Accelerometer control register to work at 104 Hz, 4 g / 16g ,and in bypass mode and enable ODR/4
	  // low pass filter (check figure9 of LSM6DS3's datasheet)
	  writeRegister(lsm, LSM6DS3_CTRL1_XL, 0x4A);// 0x4A - 4g | 0x46 - 16g
	  // 0100 1010
	  // Set the ODR config register to ODR/4
	  writeRegister(lsm, LSM6DS3_CTRL8_XL, 0x89); //0x09 - no lpf selection | 0x89 - msb turning on the low pass
	  // Set interrupt for Gyroscope data	         8421 8421
	  writeRegister(lsm, LSM6DS3_INT_CTRL , 0x02); //0000 0010


	  /*LowPass filter initialization for all axes*/
	  LowPassFilter_Init(&lsm->LpfAx, LSM6DS3_ACCEL_LPF_ALPHA);
	  LowPassFilter_Init(&lsm->LpfAy, LSM6DS3_ACCEL_LPF_ALPHA);
	  LowPassFilter_Init(&lsm->LpfAz, LSM6DS3_ACCEL_LPF_ALPHA);

	  LowPassFilter_Init(&lsm->LpfGx, LSM6DS3_GYRO_LPF_ALPHA);
	  LowPassFilter_Init(&lsm->LpfGy, LSM6DS3_GYRO_LPF_ALPHA);
	  LowPassFilter_Init(&lsm->LpfGz, LSM6DS3_GYRO_LPF_ALPHA);

	  return 0;
}

void LSM_readGyroscope(Lsm6ds3_t *lsm)
{
  int8_t value[6];
  int16_t RawData[3];

  HAL_I2C_Mem_Read(lsm->I2C, LSM6DS3_ADDRESS, LSM6DS3_OUTX_L_G , 1, (uint8_t*)value, 6, LSM6DS3_TIMEOUT);

  RawData[0] = ((value[1]<< 8 | value[0]));
  RawData[1] = ((value[3]<< 8 | value[2]));
  RawData[2] = ((value[5]<< 8 | value[4]));

  /*Conversion to degrees per second according to the ST data sheet*/
//  lsm->RawGx = (float)RawData[0] / 70.0f;
//  lsm->RawGy = (float)RawData[1] / 70.0f;
//  lsm->RawGz = (float)RawData[2] / 70.0f;
    lsm->RawGx = (float)RawData[0] * 0.07f;
    lsm->RawGy = (float)RawData[1] * 0.07f;
    lsm->RawGz = (float)RawData[2] / 70.0f;

  /*Filter the data and convert it into radians per second*/
//  lsm->Gx = (LowPassFilter_Update(&lsm->LpfGx, lsm->RawGx)) *  0.017453f;
//  lsm->Gy = (LowPassFilter_Update(&lsm->LpfGy, lsm->RawGy)) *  0.017453f;
//  lsm->Gz = (LowPassFilter_Update(&lsm->LpfGz, lsm->RawGz)) *  0.017453f;

  /*Filter the data*/
  lsm->Gx = LowPassFilter_Update(&lsm->LpfGx, lsm->RawGx);
  lsm->Gy = LowPassFilter_Update(&lsm->LpfGy, lsm->RawGy);
  lsm->Gz = LowPassFilter_Update(&lsm->LpfGz, lsm->RawGz);
}
void LSM_readAcceleration(Lsm6ds3_t *lsm)
{
  int8_t value[6];
  int16_t RawData[3];

  HAL_I2C_Mem_Read(lsm->I2C, LSM6DS3_ADDRESS, LSM6DS3_OUTX_L_XL, 1, (uint8_t*)value, 6, LSM6DS3_TIMEOUT);

  RawData[0] = ((value[1]<< 8 | value[0]));
  RawData[1] = ((value[3]<< 8 | value[2]));
  RawData[2] = ((value[5]<< 8 | value[4]));

  /*Conversion to cm/s2 according to the ST data sheet*/
  lsm->RawAx = (float)RawData[0] * 0.122f;
  lsm->RawAy = (float)RawData[1] * 0.122f;
  lsm->RawAz = (float)RawData[2] * 0.122f;

  /*Filter the data and convert it into meters per second squared*/
  lsm->Ax = (LowPassFilter_Update(&lsm->LpfAx, lsm->RawAx)) / 100.0f;
  lsm->Ay = (LowPassFilter_Update(&lsm->LpfAy, lsm->RawAy)) / 100.0f;
  lsm->Az = (LowPassFilter_Update(&lsm->LpfAz, lsm->RawAz)) / 100.0f;

}
void LSM_readTemperature(Lsm6ds3_t *lsm)
{
	  int8_t value[2];
	  int16_t RawData;

	  HAL_I2C_Mem_Read(lsm->I2C, LSM6DS3_ADDRESS, LSM6DS3_OUT_TEMP_L, 1, (uint8_t*)value, 2, LSM6DS3_TIMEOUT);

	  RawData = ((value[1]<< 8 | value[0]));

	  /*convert raw data to degrees celsius */
	  lsm->Temperature = ((float)RawData * 0.00390625f) + 25.0f;

}
