/*!
 * BMP280.h
 *
 * @Author: Yunus Emre KAYRA (github.com/YEK-Kayra)
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2023 TAISAT Turkish Artificial Intelligence Supported Autonomous Technologies
 *
 */

#ifndef INC_BMP280_H_
#define INC_BMP280_H_
#define BMP280_DEV

/****************************************************************/
/*! @name       Header includes             */
/****************************************************************/
#include "main.h"
#include "stdbool.h"
#include "stdint.h"

/****************************************************************/
/*! @name       General Macro Definitions             */
/****************************************************************/

/**!These macros provide to access the registers of BMP280 */
#define Calibration_Address_Start  0x88
#define RawDataRegStartAddress	   0xF7
#define TempRegStartAddress		   0xFA
#define BMP280_Config_REG		   0xF5 	/*! bits: t_sb[7:5] ,filter[4:2] ,spi3w_en[0]  		   */
#define BMP280_CtrlMeas_REG		   0xF4 	/*! bits: osrs_t[7:5] ,osrs_t[4:2] ,mode[1:0] 		   */
#define BMP280_Status_REG		   0xF3 	/*! bits: measuring[3] ,im_update[0]  		    	   */
#define BMP280_Reset_REG		   0xE0		/*! bits: reset[7:0] write 0xB6 into the reg for reset */

/**!These macros provide to calculate the altitude of BMP280 */
#define SeaLevelPress  101325
#define SeaLevelTemp   288.15
#define GradientTemp   0.0065
#define GravityAccel   9.80665
#define GasCoefficient 287.05

/******************************************************************************/
/*!  @name         External Variable Declarations                                  */
/******************************************************************************/
extern double BMP280_Pressure;
extern double BMP280_Temperature;
extern float  BMP280_Altitude;

/******************************************************************************/
/*!  @name         Enum Declarations                                  */
/******************************************************************************/
typedef enum {

	BMP280_Mode_Sleep  = 0,   /* no measurement          */
	BMP280_Mode_Forced = 1,   /* single measurement      */
	BMP280_Mode_Normal = 3	  /* continuous measurement  */

}BMP280_Mode;


typedef enum{

	BMP280_I2C = 0,			/* Select I2C protocol for communication */
	BMP280_SPI = 1			/* Select SPI protocol for communication */

}BMP280_Communication;


typedef enum {

	BMP280_Skipped = 0,                 		/* no measurement */
	BMP280_Ultra_Low_Power_Resolution = 1,         	/* oversampling x1 */
	BMP280_Low_Power_Resolution = 2,		/* oversampling x2 */
	BMP280_Standart_Resolution = 3, 		/* oversampling x4 */
	BMP280_High_Resolution = 4,			/* oversampling x8 */
	BMP280_Ultra_High_Resolution = 5 		/* oversampling x16 */

}BMP280_Oversampling;


typedef enum {

	BMP280_Filter_OFF = 0,      	 /* no filter */
	BMP280_Filter_X2  = 1,     	 /* filter coefficient : 2 */
	BMP280_Filter_X4  = 2,		 /* filter coefficient : 4 */
	BMP280_Filter_X8  = 3,		 /* filter coefficient : 8 */
	BMP280_Filter_X16 = 4		 /* filter coefficient : 16 */

}BMP280_Filter;


typedef enum {

	BMP280_Standby_05  = 0,         /* 0.5ms standby*/
	BMP280_Standby_62  = 1,         /* 62.5ms standby*/
	BMP280_Standby_125 = 2,         /* 125ms standby*/
	BMP280_Standby_250 = 3,         /* 250ms standby*/
	BMP280_Standby_500 = 4,         /* 500ms standby*/
	BMP280_Standby_1000 = 5,        /* 1000ms standby*/
	BMP280_Standby_2000 = 6,        /* 2000ms standby*/
	BMP280_Standby_4000 = 7         /* 4000ms standby*/

}BMP280_StandbyTime;


/******************************************************************************/
/*!  @name         Structure Declarations                             */
/******************************************************************************/

typedef struct {

	BMP280_Mode   Mode;
	BMP280_Oversampling Oversampling_Pressure;
	BMP280_Oversampling Oversampling_Temperature;
	BMP280_Filter Filter;
	BMP280_StandbyTime StandbyTime;
	BMP280_Communication CommunicationInterface;

}BMP280_Params_t;


typedef struct {

	/**!dig_T1 to dig_T3 and dig_P1 to dig_P9 are variable that we can save calibration parameters */
	uint16_t dig_T1;
	int16_t  dig_T2;
	int16_t  dig_T3;
	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;

	BMP280_Params_t BMP280_Params;

	uint16_t BMP280_I2C_ADDRESS;
	I2C_HandleTypeDef *i2c;

	/**! Each variable keep 2-3 data about its register.
	 *  For instance "BMP280_Config_Params_t" contains that:
	 * 							 BMP280_Params.StandbyTime
	 * 							 BMP280_Params.Filter
	 * 							 BMP280_Params.CommunicationInterface
	 * */
	uint8_t  BMP280_Config_Params_t;
	uint8_t	 BMP280_CtrlMeas_Params_t;
    uint8_t  BMP280_Status;



}BMP280_HandleTypeDef;


/******************************************************************************/
/*         			BMP280 Function prototypes            	*/
/******************************************************************************/

/**
 * @fn      void BMP280_Init(BMP280_HandleTypeDef *BMP280)
 * @brief   Get calibration datas from BMP280's chip
 * @param   *BMP280
 * @return  None
 */
void BMP280_Get_CalibrationDatas(BMP280_HandleTypeDef *BMP280);


/**
 * @fn      void BMP280_Set_DefaultParams(BMP280_Params_t *BMP280_Params)
 * @brief   Select BMP280 configurations. The values can be changed by users
 * @param   *BMP280_Params
 * @retval  None
 */
void BMP280_Set_DefaultParams(BMP280_Params_t *BMP280_Params);


/**
 * @fn      void BMP280_Init(BMP280_HandleTypeDef *BMP280)
 * @brief   Uploads BMP280 configuration parameters into the BMP280 and start BMP280
 * @param   *BMP280
 * @return  None
 */
void BMP280_Init(BMP280_HandleTypeDef *BMP280);


/**
 * @fn       bool BMP280_Control_Status(BMP280_HandleTypeDef *BMP280)
 * @brief    Checks the status register for operations of Copying value and Updating
 * @param    *BMP280
 * @return   boolean number according to sensor status
 * @retval   '1' : The values are ready to get from BMP280
 * 	     '0' : The values are not ready to get from BMP280 and it will wait until sensor is ready
 */
bool BMP280_Control_Status(BMP280_HandleTypeDef *BMP280);



/**
 * @fn      double BMP280_GetVal_TemperatureAndPressure(BMP280_HandleTypeDef *BMP280, double *BMP280_Temperature, double *BMP280_Pressure)
 * @brief   Read rawValues for Temperature and Pressure from BMP280's chip registers.
 * 	    And it is sent raw values to other function to calculate their real values
 *
 * @param   *BMP280
 * @param   *BMP280_Temperature
 * @param   *BMP280_Pressure
 * @return  boolean number according to sensor status
 * @retval  '1' : The values are ready to get from BMP280
 * 	    '0' : The values are not ready to get from BMP280 and it will wait until sensor is ready
 */
bool BMP280_GetVal_TemperatureAndPressure(BMP280_HandleTypeDef *BMP280, double *BMP280_Temperature, double *BMP280_Pressure, float *BMP280_Altitude);


/**
 * @fn      double BMP280_Calculate_CompensatedTemperature(BMP280_HandleTypeDef *BMP280, int32_t RawTemperature, int32_t *TempVarTotel)
 * @brief   Calculate the Compensated Temperature by calibration datas
 * @param   *BMP280
 * @param   RawTemperature
 * @param   *TempVarTotel
 * @return  Compensated Temperature
 */
double BMP280_Calculate_CompensatedTemperature(BMP280_HandleTypeDef *BMP280, int32_t RawTemperature, int32_t *TempVarTotel);

/**
 * @fn      double BMP280_Calculate_CompensatedPressure(BMP280_HandleTypeDef *BMP280, int32_t RawPressure,int32_t TempVarTotel)
 * @brief   Calculate the Compensated Pressure by calibration datas
 * @param   *BMP280
 * @param   RawPressure
 * @param   TempVarTotel
 * @return  Compensated Pressure
 */
double BMP280_Calculate_CompensatedPressure(BMP280_HandleTypeDef *BMP280, int32_t RawPressure, int32_t TempVarTotel);

/**
 * @fn      float BMP280_Calculate_Altitude(double *BMP280_Pressure)
 * @brief   Calculate the Altitude by BMP280_Pressure
 * @param   *BMP280_Pressure
 * @return  Altitude
 */
float BMP280_Calculate_Altitude(double *BMP280_Pressure);

#endif /* INC_BMP280_H_ */
