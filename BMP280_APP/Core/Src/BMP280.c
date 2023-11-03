/*!
 *  @file : BMP280.c
 *  @date : 8-10-2023
 *  @version : v1.0.0
 *
 *      Author: Yunus Emre KAYRA (https://github.com/YEK-Kayra)
 ******************************************************************************
 * 	@attention
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2023 TAISAT Turkish Artificial Intelligence Supported Autonomous Technologies
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */



/****************************************************************/
/*! @name      	 Header includes                */
/****************************************************************/
#include "BMP280.h"
#include "stm32f1xx.h"
#include "main.h"
#include "stdint.h"
#include "math.h"


/******************************************************************************/
/*! @name        BMP280  Function prototypes            				*/
/******************************************************************************/

/**!These variable provide to calculate the altitude of BMP280 */
float FixedAltitude;
float TemporaryAltitude;


void BMP280_Init(BMP280_HandleTypeDef *BMP280){

	BMP280_Get_CalibrationDatas(BMP280);

	BMP280->BMP280_CtrlMeas_Params_t = (BMP280->BMP280_Params.Oversampling_Temperature<<5) |
			   	   	   	   	   	   	   (BMP280->BMP280_Params.Oversampling_Pressure<<2) |
									   (BMP280->BMP280_Params.Mode<<0);


    BMP280->BMP280_Config_Params_t 	 = (BMP280->BMP280_Params.StandbyTime<<5) |
			   	   	   	   	   	   	   (BMP280->BMP280_Params.Filter<<2) |
									   (BMP280->BMP280_Params.CommunicationInterface<<0);


    HAL_I2C_Mem_Write(BMP280->i2c, BMP280->BMP280_I2C_ADDRESS, BMP280_CtrlMeas_REG, 1, &BMP280->BMP280_CtrlMeas_Params_t, 1, 1000);
    HAL_I2C_Mem_Write(BMP280->i2c, BMP280->BMP280_I2C_ADDRESS, BMP280_Config_REG, 1, &BMP280->BMP280_Config_Params_t, 1, 1000);

    for(int cnt = 0 ; cnt < 20 ; cnt++){

    	BMP280_GetVal_TemperatureAndPressure(BMP280, &BMP280_Temperature, &BMP280_Pressure, &BMP280_Altitude);
    	TemporaryAltitude = (float)(TemporaryAltitude + (float)(BMP280_Altitude * (0.05)));

    }

    FixedAltitude = TemporaryAltitude;
}


void BMP280_Get_CalibrationDatas(BMP280_HandleTypeDef *BMP280){

	uint8_t BMP280_CalibrationDatas[24] = {0};
	uint8_t CNT = 0;

	if(HAL_I2C_IsDeviceReady(BMP280->i2c, BMP280->BMP280_I2C_ADDRESS, 1, 500) != HAL_OK){

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);

	}

	HAL_I2C_Mem_Read(BMP280->i2c, BMP280->BMP280_I2C_ADDRESS, Calibration_Address_Start, 1, &BMP280_CalibrationDatas[0], 24, 1000);

	BMP280->dig_T1 = (uint16_t)((BMP280_CalibrationDatas[CNT]) | (BMP280_CalibrationDatas[CNT+1]<<8)); CNT+=2;
	BMP280->dig_T2 =  (int16_t)((BMP280_CalibrationDatas[CNT]) | (BMP280_CalibrationDatas[CNT+1]<<8)); CNT+=2;
	BMP280->dig_T3 =  (int16_t)((BMP280_CalibrationDatas[CNT]) | (BMP280_CalibrationDatas[CNT+1]<<8)); CNT+=2;
	BMP280->dig_P1 = (uint16_t)((BMP280_CalibrationDatas[CNT]) | (BMP280_CalibrationDatas[CNT+1]<<8)); CNT+=2;
	BMP280->dig_P2 =  (int16_t)((BMP280_CalibrationDatas[CNT]) | (BMP280_CalibrationDatas[CNT+1]<<8)); CNT+=2;
	BMP280->dig_P3 =  (int16_t)((BMP280_CalibrationDatas[CNT]) | (BMP280_CalibrationDatas[CNT+1]<<8)); CNT+=2;
	BMP280->dig_P4 =  (int16_t)((BMP280_CalibrationDatas[CNT]) | (BMP280_CalibrationDatas[CNT+1]<<8)); CNT+=2;
	BMP280->dig_P5 =  (int16_t)((BMP280_CalibrationDatas[CNT]) | (BMP280_CalibrationDatas[CNT+1]<<8)); CNT+=2;
	BMP280->dig_P6 =  (int16_t)((BMP280_CalibrationDatas[CNT]) | (BMP280_CalibrationDatas[CNT+1]<<8)); CNT+=2;
	BMP280->dig_P7 =  (int16_t)((BMP280_CalibrationDatas[CNT]) | (BMP280_CalibrationDatas[CNT+1]<<8)); CNT+=2;
	BMP280->dig_P8 =  (int16_t)((BMP280_CalibrationDatas[CNT]) | (BMP280_CalibrationDatas[CNT+1]<<8)); CNT+=2;
	BMP280->dig_P9 =  (int16_t)((BMP280_CalibrationDatas[CNT]) | (BMP280_CalibrationDatas[CNT+1]<<8)); CNT+=2;

}


void BMP280_Set_DefaultParams(BMP280_Params_t *BMP280_Params){

	BMP280_Params->CommunicationInterface = BMP280_I2C;
	BMP280_Params->Filter = BMP280_Filter_X16;
	BMP280_Params->StandbyTime = BMP280_Standby_05;
	BMP280_Params->Mode = BMP280_Mode_Normal;
	BMP280_Params->Oversampling_Pressure = BMP280_Ultra_High_Resolution;
	BMP280_Params->Oversampling_Temperature = BMP280_Ultra_High_Resolution;

}




bool BMP280_GetVal_TemperatureAndPressure(BMP280_HandleTypeDef *BMP280, double *BMP280_Temperature, double *BMP280_Pressure, float *BMP280_Altitude){

	uint8_t PressTemp_MSB_LSB_XLSB[6] = {0};
	int32_t RawTemperature;
	int32_t RawPressure;
	int32_t TempVarTotel;


	if(BMP280_Control_Status(BMP280) == true){


		HAL_I2C_Mem_Read(BMP280->i2c, BMP280->BMP280_I2C_ADDRESS, RawDataRegStartAddress, 1, PressTemp_MSB_LSB_XLSB, 6, 1000);

		RawTemperature = (PressTemp_MSB_LSB_XLSB[3]<<12 | PressTemp_MSB_LSB_XLSB[4]<<4 | PressTemp_MSB_LSB_XLSB[5]>>4);
		RawPressure = (PressTemp_MSB_LSB_XLSB[0]<<12 | PressTemp_MSB_LSB_XLSB[1]<<4 | PressTemp_MSB_LSB_XLSB[2]>>4);


		*BMP280_Temperature = (float)BMP280_Calculate_CompensatedTemperature(BMP280, RawTemperature,&TempVarTotel);
		*BMP280_Pressure = (float)BMP280_Calculate_CompensatedPressure(BMP280, RawPressure, TempVarTotel);
		*BMP280_Altitude = (float)BMP280_Calculate_Altitude(BMP280_Pressure);

		return 1;

	}
	else{

		return 0;

	}

}


float BMP280_Calculate_Altitude(double *BMP280_Pressure){

	return ((SeaLevelTemp / GradientTemp) * (1 - pow((*BMP280_Pressure / SeaLevelPress),((GasCoefficient * GradientTemp)/GravityAccel)))-FixedAltitude);

}


double BMP280_Calculate_CompensatedTemperature(BMP280_HandleTypeDef *BMP280, int32_t RawTemperature, int32_t *TempVarTotel){

	int32_t var1, var2;

	var1 = ((((RawTemperature >> 3) - ((int32_t) BMP280->dig_T1 << 1)))
			* (int32_t) BMP280->dig_T2) >> 11;

	var2 = (((((RawTemperature >> 4) - (int32_t) BMP280->dig_T1)
			* ((RawTemperature >> 4) - (int32_t) BMP280->dig_T1)) >> 12)
			* (int32_t) BMP280->dig_T3) >> 14;

	*TempVarTotel = var1 + var2;
	return ((float)((*TempVarTotel * 5 + 128) >> 8)/100);

}


double BMP280_Calculate_CompensatedPressure(BMP280_HandleTypeDef *BMP280, int32_t RawPressure,int32_t TempVarTotel){

	int64_t var1, var2, p;

	var1 = (int64_t) TempVarTotel - 128000;
	var2 = var1 * var1 * (int64_t) BMP280->dig_P6;
	var2 = var2 + ((var1 * (int64_t) BMP280->dig_P5) << 17);
	var2 = var2 + (((int64_t) BMP280->dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t) BMP280->dig_P3) >> 8)
			+ ((var1 * (int64_t) BMP280->dig_P2) << 12);
	var1 = (((int64_t) 1 << 47) + var1) * ((int64_t) BMP280->dig_P1) >> 33;

	if (var1 == 0) {
		return 0;  // avoid exception caused by division by zero
	}

	p = 1048576 - RawPressure;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = ((int64_t) BMP280->dig_P9 * (p >> 13) * (p >> 13)) >> 25;
	var2 = ((int64_t) BMP280->dig_P8 * p) >> 19;

	p = ((p + var1 + var2) >> 8) + ((int64_t) BMP280->dig_P7 << 4);
	return ((float)p/256);

}


bool BMP280_Control_Status(BMP280_HandleTypeDef *BMP280){

	HAL_I2C_Mem_Read(BMP280->i2c, BMP280->BMP280_I2C_ADDRESS, BMP280_Status_REG, 1, &BMP280->BMP280_Status, 1, 1000);


	while(!(BMP280->BMP280_Status & 0x01)){

		HAL_I2C_Mem_Read(BMP280->i2c, BMP280->BMP280_I2C_ADDRESS, BMP280_Status_REG, 1, &BMP280->BMP280_Status, 1, 1000);

	}

	return true;

}

