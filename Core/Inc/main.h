/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tim.h"
#include "dma.h"
#include "fsmc.h"
#include "usart.h"
#include "math.h"
#include "stdbool.h"
#include "stdlib.h"
#include "PID.h"
#include "Sensor.h"
#include "LCD.h"
#include "LCD_FONT.h"
#include "Moto.h"
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
#define SENSOR01_Pin GPIO_PIN_2
#define SENSOR01_GPIO_Port GPIOE
#define SENSOR02_Pin GPIO_PIN_3
#define SENSOR02_GPIO_Port GPIOE
#define SENSOR03_Pin GPIO_PIN_4
#define SENSOR03_GPIO_Port GPIOE
#define SENSOR04_Pin GPIO_PIN_5
#define SENSOR04_GPIO_Port GPIOE
#define SENSOR05_Pin GPIO_PIN_6
#define SENSOR05_GPIO_Port GPIOE
#define SENSOR06_Pin GPIO_PIN_13
#define SENSOR06_GPIO_Port GPIOC
#define SENSOR07_Pin GPIO_PIN_0
#define SENSOR07_GPIO_Port GPIOC
#define SENSOR08_Pin GPIO_PIN_1
#define SENSOR08_GPIO_Port GPIOC
#define SENSOR09_Pin GPIO_PIN_2
#define SENSOR09_GPIO_Port GPIOC
#define SENSOR10_Pin GPIO_PIN_3
#define SENSOR10_GPIO_Port GPIOC
#define ENCODERD0_Pin GPIO_PIN_0
#define ENCODERD0_GPIO_Port GPIOA
#define ENCODERD1_Pin GPIO_PIN_1
#define ENCODERD1_GPIO_Port GPIOA
#define SENSOR11_Pin GPIO_PIN_4
#define SENSOR11_GPIO_Port GPIOA
#define SENSOR12_Pin GPIO_PIN_5
#define SENSOR12_GPIO_Port GPIOA
#define SENSOR13_Pin GPIO_PIN_6
#define SENSOR13_GPIO_Port GPIOA
#define SENSOR14_Pin GPIO_PIN_7
#define SENSOR14_GPIO_Port GPIOA
#define SENSOR15_Pin GPIO_PIN_4
#define SENSOR15_GPIO_Port GPIOC
#define SENSOR16_Pin GPIO_PIN_5
#define SENSOR16_GPIO_Port GPIOC
#define SENSOR17_Pin GPIO_PIN_0
#define SENSOR17_GPIO_Port GPIOB
#define SENSOR18_Pin GPIO_PIN_1
#define SENSOR18_GPIO_Port GPIOB
#define SENSOR19_Pin GPIO_PIN_2
#define SENSOR19_GPIO_Port GPIOB
#define SENSOR20_Pin GPIO_PIN_11
#define SENSOR20_GPIO_Port GPIOF
#define SENSOR21_Pin GPIO_PIN_12
#define SENSOR21_GPIO_Port GPIOF
#define SENSOR22_Pin GPIO_PIN_13
#define SENSOR22_GPIO_Port GPIOF
#define SENSOR23_Pin GPIO_PIN_14
#define SENSOR23_GPIO_Port GPIOF
#define SENSOR24_Pin GPIO_PIN_15
#define SENSOR24_GPIO_Port GPIOF
#define LCD_BL_Pin GPIO_PIN_1
#define LCD_BL_GPIO_Port GPIOG
#define IIC3_SCL_Pin GPIO_PIN_12
#define IIC3_SCL_GPIO_Port GPIOB
#define IIC3_SDA_Pin GPIO_PIN_13
#define IIC3_SDA_GPIO_Port GPIOB
#define IIC2_SDA_Pin GPIO_PIN_14
#define IIC2_SDA_GPIO_Port GPIOB
#define IIC2_SCL_Pin GPIO_PIN_15
#define IIC2_SCL_GPIO_Port GPIOB
#define IIC1_SDA_Pin GPIO_PIN_11
#define IIC1_SDA_GPIO_Port GPIOD
#define IIC1_SCL_Pin GPIO_PIN_12
#define IIC1_SCL_GPIO_Port GPIOD
#define MOTO_POWER_Pin GPIO_PIN_13
#define MOTO_POWER_GPIO_Port GPIOD
#define MOTOA0_Pin GPIO_PIN_2
#define MOTOA0_GPIO_Port GPIOG
#define MOTOA1_Pin GPIO_PIN_3
#define MOTOA1_GPIO_Port GPIOG
#define MOTOB0_Pin GPIO_PIN_4
#define MOTOB0_GPIO_Port GPIOG
#define MOTOB1_Pin GPIO_PIN_5
#define MOTOB1_GPIO_Port GPIOG
#define MOTOC0_Pin GPIO_PIN_6
#define MOTOC0_GPIO_Port GPIOG
#define MOTOC1_Pin GPIO_PIN_7
#define MOTOC1_GPIO_Port GPIOG
#define MOTOD0_Pin GPIO_PIN_8
#define MOTOD0_GPIO_Port GPIOG
#define MOTOA_Pin GPIO_PIN_6
#define MOTOA_GPIO_Port GPIOC
#define MOTOB_Pin GPIO_PIN_7
#define MOTOB_GPIO_Port GPIOC
#define MOTOC_Pin GPIO_PIN_8
#define MOTOC_GPIO_Port GPIOC
#define MOTOD_Pin GPIO_PIN_9
#define MOTOD_GPIO_Port GPIOC
#define MOTOD1_Pin GPIO_PIN_8
#define MOTOD1_GPIO_Port GPIOA
#define START_BTN_Pin GPIO_PIN_10
#define START_BTN_GPIO_Port GPIOG
#define ENCODERA0_Pin GPIO_PIN_15
#define ENCODERA0_GPIO_Port GPIOA
#define ENCODERA1_Pin GPIO_PIN_3
#define ENCODERA1_GPIO_Port GPIOB
#define ENCODERB0_Pin GPIO_PIN_4
#define ENCODERB0_GPIO_Port GPIOB
#define ENCODERB1_Pin GPIO_PIN_5
#define ENCODERB1_GPIO_Port GPIOB
#define ENCODERC0_Pin GPIO_PIN_6
#define ENCODERC0_GPIO_Port GPIOB
#define ENCODERC1_Pin GPIO_PIN_7
#define ENCODERC1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
