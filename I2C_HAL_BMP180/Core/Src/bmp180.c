/*
 * bmp180.c
 *
 *
 *      Author: erkut
 *      github:erktaksu
 */


#include "bmp180.h"
#include <math.h>

short AC1=0;
short AC2=0;
short AC3=0;
unsigned short AC4=0;
unsigned short AC5=0;
unsigned short AC6=0;
short B1=0;
short B2=0;
short MB=0;
short MC=0;
short MD=0;

//temperature

long X1=0;
long X2=0;
long B5=0;
float temperature=0;

//Pressure
long B6=0;
long X3=0;
long B3=0;
unsigned long B4=0;
unsigned long B7=0;
long preassure=0;

long UT;
long UP;

float preassureATM=0;

void BMP180_Init() {
    if (HAL_I2C_IsDeviceReady(&hi2c1, BMP180_DEVICE_WRITE_REGISTER_ADDRESS, 1, 100000) != HAL_OK)
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);

    BMP180_GetCalibration_Value();
}

void BMP180_GetCalibration_Value() {
    uint8_t CalibBuff[BMP180_CALIBRATION_VALUE_LENGTH];
    HAL_I2C_Mem_Read(&hi2c1, BMP180_DEVICE_READ_REGISTER_ADDRESS, BMP180_CALIBRATION_START_ADRESS, 1, CalibBuff, BMP180_CALIBRATION_VALUE_LENGTH, 100);

    AC1 = ((CalibBuff[0] << 8) | CalibBuff[1]);
    AC2 = ((CalibBuff[2] << 8) | CalibBuff[3]);
    AC3 = ((CalibBuff[4] << 8) | CalibBuff[5]);
    AC4 = ((CalibBuff[6] << 8) | CalibBuff[7]);
    AC5 = ((CalibBuff[8] << 8) | CalibBuff[9]);
    AC6 = ((CalibBuff[10] << 8) | CalibBuff[11]);
    B1 = ((CalibBuff[12] << 8) | CalibBuff[13]);
    B2 = ((CalibBuff[14] << 8) | CalibBuff[15]);
    MB = ((CalibBuff[16] << 8) | CalibBuff[17]);
    MC = ((CalibBuff[18] << 8) | CalibBuff[19]);
    MD = ((CalibBuff[20] << 8) | CalibBuff[21]);
}

float BMP180_Get_Temperature(void) {
    UT=BMP180_Get_Uncompensated_Temperature();
    X1 = ((UT - AC6) * AC5) / pow(2,15);
     X2 = (MC * 2048) / (X1 + MD);
    B5 = X1 + X2;
    temperature = ((float)(B5 + 8) / pow(2,4)) * 0.1;
return temperature;
}

long BMP180_Get_Uncompensated_Temperature(void) {
    uint8_t rData[2];
    uint8_t wData[1] = {0x2E};
    HAL_I2C_Mem_Write(&hi2c1, BMP180_DEVICE_WRITE_REGISTER_ADDRESS, 0xF4, 1, wData, 1, 1000);
    HAL_Delay(5);
    HAL_I2C_Mem_Read(&hi2c1, BMP180_DEVICE_READ_REGISTER_ADDRESS, 0xF6, 1, rData, 2, 1000);
    return( (int16_t)((rData[0] << 8) | rData[1]));

}

float BMP180_Get_Pressure(void) {
    UP=BMP180_Get_Uncompensated_Pressure();
    B6=B5-4000;
    X1=(B2*(B6*B6/pow(2,12)))/pow(2,11);
    X2=AC2*B6/pow(2,11);
    X3=X1+X2;
    B3=(((AC1*4+X3)<<3)+2)/4;
    X1=AC3*B6/pow(2,13);
    X2=(B1*(B6*B6/pow(2,12)))/pow(2,16);
    X3=((X1+X2)+2)/pow(2,2);
    B4=AC4*(unsigned long)(X3+32768)/pow(2,15);
    B7=((unsigned long)UP-B3)*(50000>>3);

    if(B7<0x80000000)
    {
    	preassure=(B7*2)/B4;
    }
    else
    {
    	preassure=(B7/B4)*2;
    }


    X1=(preassure/pow(2,8))*(preassure/pow(2,8));
    X1=(X1*3038)/pow(2,16);
    X2=(-7357*preassure)/pow(2,16);
    preassure=preassure+(X1+X2+3791)/pow(2,4);

    preassureATM= (float)preassure*0.00000987;
return     preassureATM;
}

long BMP180_Get_Uncompensated_Pressure(void) {
    uint8_t wData1[1] = {0};
    uint8_t rData1[3]={0};
    wData1[0]= 0x34 | (3 << 6);
    HAL_I2C_Mem_Write(&hi2c1, BMP180_DEVICE_WRITE_REGISTER_ADDRESS, 0xF4, 1, wData1, 1, 1000);
    HAL_Delay(26);
    HAL_I2C_Mem_Read(&hi2c1, BMP180_DEVICE_READ_REGISTER_ADDRESS, 0xF6, 1, rData1, 3, 1000);
    return((((rData1[0] << 16) + (rData1[1] << 8) + rData1[2]) >> (8 - 3)));

}
