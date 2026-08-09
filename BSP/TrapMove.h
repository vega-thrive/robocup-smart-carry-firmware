#ifndef __TRAP_MOVE_H__
#define __TRAP_MOVE_H__

#include "main.h"

#define TRAP_ACCEL           120
#define TRAP_DECEL           25
#define TRAP_MIN_SPEED       200
#define TRAP_ERROR_THRESHOLD 600

typedef struct
{
    int32_t target;
    int32_t position;
    int cmd_speed;
    int accel;
    int decel;
    int min_speed;
    int error_threshold;
} TrapMove_Planner;

void TrapMove_Init(TrapMove_Planner *tp, int accel, int decel,
                   int min_speed, int error_threshold);
void TrapMove_Reset(TrapMove_Planner *tp, int32_t target);
int TrapMove_Update(TrapMove_Planner *tp, int32_t delta, int max_speed);
int TrapMove_GetError(TrapMove_Planner *tp);

#endif
