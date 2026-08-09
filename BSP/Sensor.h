#ifndef SENSOR_H
#define SENSOR_H

#include "main.h"

/* 前八路灰度 (中心巡线用 SENSOR01~08) */
extern uint8_t GS_state[9];       /* 索引 1~8 */
void Sensor_Get(void);

/* 后八路灰度 (PB0/1/2, PF11~15) */
extern uint8_t Rear_GS_state[9];  /* 索引 1~8 */
void RearSensor_Get(void);

#endif
