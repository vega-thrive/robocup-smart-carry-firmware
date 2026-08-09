#ifndef __HWT101_H__
#define __HWT101_H__

#include "main.h"

extern volatile float Angle;
void HWT101_getAngle_Reset(void);
void HWT101_UART_Init(void);
void HWT101_UART_RxCpltCallback(UART_HandleTypeDef *huart);
uint8_t HWT101_IsReady(void);
uint32_t HWT101_GetFrameAgeMs(void);
uint32_t HWT101_GetUpdateCount(void);
float HWT101_GetAbsoluteYaw(void);



#endif

