#ifndef __TASK1_COLOR_H__
#define __TASK1_COLOR_H__

#include "main.h"

extern volatile uint8_t task1_color_prepare_pending;
extern volatile uint8_t task1_color_start_pending;

void Task1Color_Start(void);
void Task1Color_Stop(void);
void Task1Color_Process(void);
void Task1Color_ServiceCommands(void);

#endif
