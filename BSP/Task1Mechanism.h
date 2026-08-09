#ifndef __TASK1_MECHANISM_H__
#define __TASK1_MECHANISM_H__

#include <stdint.h>

/* Independent Task 1 mechanism working pose. */
typedef struct
{
    uint16_t big_arm_us;
    uint16_t small_arm_us;
} Task1ArmPose;

static inline const Task1ArmPose *Task1Mechanism_GetWorkingPose(void)
{
    static const Task1ArmPose working_pose =
    {
        1250U, /* big arm: CH1 / PC2 */
        1500U  /* small arm: CH2 / PC3 */
    };
    return &working_pose;
}

#endif
