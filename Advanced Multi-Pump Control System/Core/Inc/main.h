/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Fpresent_LED_Pin GPIO_PIN_3
#define Fpresent_LED_GPIO_Port GPIOE
#define ESP32_RST_Pin GPIO_PIN_4
#define ESP32_RST_GPIO_Port GPIOE
#define ESP32_IO0_Pin GPIO_PIN_5
#define ESP32_IO0_GPIO_Port GPIOE
#define ADC_in2_Pin GPIO_PIN_0
#define ADC_in2_GPIO_Port GPIOC
#define Din4_Pin GPIO_PIN_1
#define Din4_GPIO_Port GPIOC
#define ADC_in1_Pin GPIO_PIN_0
#define ADC_in1_GPIO_Port GPIOA
#define Din3_Pin GPIO_PIN_7
#define Din3_GPIO_Port GPIOA
#define Din1_Pin GPIO_PIN_4
#define Din1_GPIO_Port GPIOC
#define Din2_Pin GPIO_PIN_5
#define Din2_GPIO_Port GPIOC
#define HWid2_Pin GPIO_PIN_11
#define HWid2_GPIO_Port GPIOB
#define HWid0_Pin GPIO_PIN_12
#define HWid0_GPIO_Port GPIOB
#define HWid1_Pin GPIO_PIN_13
#define HWid1_GPIO_Port GPIOB
#define LED_R_Pin GPIO_PIN_13
#define LED_R_GPIO_Port GPIOD
#define LED_G_Pin GPIO_PIN_14
#define LED_G_GPIO_Port GPIOD
#define LED_B_Pin GPIO_PIN_15
#define LED_B_GPIO_Port GPIOD
#define Dout1_Pin GPIO_PIN_0
#define Dout1_GPIO_Port GPIOD
#define Dout2_Pin GPIO_PIN_1
#define Dout2_GPIO_Port GPIOD
#define Dout3_Pin GPIO_PIN_2
#define Dout3_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
