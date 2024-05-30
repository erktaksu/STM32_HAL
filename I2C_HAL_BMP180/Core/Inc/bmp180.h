/*
 * bmp180.h
 *

 *      Author: erkut
 *      github:erktaksu
 */

#ifndef INC_BMP180_H_
#define INC_BMP180_H_

#include "stm32f7xx_hal.h"
#include <stdint.h>

extern I2C_HandleTypeDef hi2c1;
//bmp180 device adres
#define BMP180_DEVICE_WRITE_REGISTER_ADDRESS 0XEE
#define BMP180_DEVICE_READ_REGISTER_ADDRESS 0XEF
//BMP180 Calibration start adres

#define BMP180_CALIBRATION_START_ADRESS 0XAA

//bmp180 calibration value length

#define BMP180_CALIBRATION_VALUE_LENGTH 22


void BMP180_Init(void);
void BMP180_GetCalibrition(void);
void BMP180_GetCalibration_Value(void);
long BMP180_Get_Uncompensated_Temperature(void);
float BMP180_Get_Temperature(void);
long BMP180_Get_Uncompensated_Pressure(void);
float BMP180_Get_Pressure(void);
#endif /* INC_BMP180_H_ */
