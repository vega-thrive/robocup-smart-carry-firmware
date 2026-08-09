#ifndef __TROPHY_MECHANISM_H__
#define __TROPHY_MECHANISM_H__

#include <stdint.h>

/* Task 2 trophy mechanism poses. */
typedef struct
{
    uint16_t big_arm_us;
    uint16_t small_arm_us;
} TrophyArmPose;

/*
 * Header-only for now so the first calibrated pose works even when an older
 * Keil target list does not yet contain a new TrophyMechanism.c source file.
 */
static inline const TrophyArmPose *TrophyMechanism_GetReadyPose(void)
{
    static const TrophyArmPose ready_pose =
    {
        900U,  /* big arm: CH0 / PC2 */
        1000U  /* small arm: CH1 / PC3 */
    };
    return &ready_pose;
}

#endif
