#ifndef __VISION_ALIGNMENT_H__
#define __VISION_ALIGNMENT_H__

#include "main.h"

typedef enum
{
    VISION_ALIGN_IDLE = 0,
    VISION_ALIGN_RUNNING,
    VISION_ALIGN_DONE,
    VISION_ALIGN_TIMEOUT
} VisionAlignment_Status;

void VisionAlignment_Init(UART_HandleTypeDef *uart);
void VisionAlignment_Start(void);
void VisionAlignment_Process(void);
void VisionAlignment_Cancel(void);
VisionAlignment_Status VisionAlignment_GetStatus(void);
void VisionAlignment_GetWheelTargets(int target[4]);

#endif
