/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MOTO_POWER_GPIO_Port, MOTO_POWER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, MOTOA0_Pin|MOTOA1_Pin|MOTOB0_Pin|MOTOB1_Pin
                          |MOTOC0_Pin|MOTOC1_Pin|MOTOD0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MOTOD1_GPIO_Port, MOTOD1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SENSOR01_Pin SENSOR02_Pin SENSOR03_Pin SENSOR04_Pin
                           SENSOR05_Pin */
  GPIO_InitStruct.Pin = SENSOR01_Pin|SENSOR02_Pin|SENSOR03_Pin|SENSOR04_Pin
                          |SENSOR05_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : SENSOR06_Pin SENSOR07_Pin SENSOR08_Pin SENSOR09_Pin
                           SENSOR10_Pin SENSOR15_Pin SENSOR16_Pin */
  GPIO_InitStruct.Pin = SENSOR06_Pin|SENSOR07_Pin|SENSOR08_Pin|SENSOR09_Pin
                          |SENSOR10_Pin|SENSOR15_Pin|SENSOR16_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SENSOR11_Pin SENSOR12_Pin SENSOR13_Pin SENSOR14_Pin */
  GPIO_InitStruct.Pin = SENSOR11_Pin|SENSOR12_Pin|SENSOR13_Pin|SENSOR14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SENSOR17_Pin SENSOR18_Pin SENSOR19_Pin */
  GPIO_InitStruct.Pin = SENSOR17_Pin|SENSOR18_Pin|SENSOR19_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SENSOR20_Pin SENSOR21_Pin SENSOR22_Pin SENSOR23_Pin
                           SENSOR24_Pin */
  GPIO_InitStruct.Pin = SENSOR20_Pin|SENSOR21_Pin|SENSOR22_Pin|SENSOR23_Pin
                          |SENSOR24_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_BL_Pin MOTOA1_Pin MOTOB0_Pin MOTOB1_Pin
                           MOTOC0_Pin MOTOC1_Pin MOTOD0_Pin */
  GPIO_InitStruct.Pin = LCD_BL_Pin|MOTOA1_Pin|MOTOB0_Pin|MOTOB1_Pin
                          |MOTOC0_Pin|MOTOC1_Pin|MOTOD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* Released legacy software-I2C pins: PB12..PB15. */
  GPIO_InitStruct.Pin = IIC3_SCL_Pin|IIC3_SDA_Pin|IIC2_SDA_Pin|IIC2_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Released legacy software-I2C pins: PD11..PD12. */
  GPIO_InitStruct.Pin = IIC1_SDA_Pin|IIC1_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : MOTO_POWER_Pin */
  GPIO_InitStruct.Pin = MOTO_POWER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOTO_POWER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MOTOA0_Pin */
  GPIO_InitStruct.Pin = MOTOA0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOTOA0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MOTOD1_Pin */
  GPIO_InitStruct.Pin = MOTOD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOTOD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : START_BTN (PG10) — active-low button */
  GPIO_InitStruct.Pin = START_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(START_BTN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
