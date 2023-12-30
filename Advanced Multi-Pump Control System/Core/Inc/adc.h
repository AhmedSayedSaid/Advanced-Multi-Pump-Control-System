/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;

extern ADC_HandleTypeDef hadc2;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_ADC1_Init(void);
void MX_ADC2_Init(void);

/* USER CODE BEGIN Prototypes */
//varibles that will is used to calibrate the adc
extern float coffA;
extern float coffB;
extern float cutIn;
extern float cutOff;
extern float currentOfTheFirstReading;
extern float currentOfTheSecondReading;
extern float minimumPressure;
extern float MaximumPressure;
extern uint8_t enableDryRun;
extern uint16_t firstADCCalibrationReading;
extern uint16_t secondADCCalibrationReading;

/**
 * @brief Calibrates the Analog-to-Digital Converter (ADC).
 *
 * This function computes the calibration coefficients (coffA and coffB) for the ADC using the first and second calibration readings.
 * The coefficients are used to linearly interpolate between the ADC readings and the corresponding current values.
 *
 * Preconditions:
 * - The variables currentOfTheFirstReading, currentOfTheSecondReading, firstADCCalibrationReading, and secondADCCalibrationReading
 *   must be initialized with the calibration data obtained from the ADC.
 * - The secondADCCalibrationReading must be greater than the firstADCCalibrationReading to avoid division by zero.
 *
 * Postconditions:
 * - The global variables coffA and coffB are set with the calculated calibration coefficients.
 *
 * @note This function does not return any value. It updates global variables directly.
 */
void CalibrateADC();
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

