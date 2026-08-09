#include "LineFollow.h"
#include "HWT101.h"
#include "HeadingHold.h"
#include "ChassisPos.h"

#define LINE_FOLLOW_KP          9.0f
#define LINE_FOLLOW_KI          0.15f
#define LINE_FOLLOW_KD          0.8f
#define LINE_FOLLOW_I_LIMIT     200

/* 八路灰度权重，索引1~8，对应传感器物理位置从左到右。
 * 循迹算法：加权位置和 → 位置式PID → 左右列差速转向。
 * line_state > 0 = 车偏左/线在右，turn > 0，左列加速/右列减速 = 车头右偏找线。 */
const int8_t line_weights[9] = {0, -12, -9, -6, -3, 3, 6, 9, 12};

volatile int wheel_target[5] = {0};
static float linefollow_pid_integral;
static int   linefollow_pid_last_error;
static uint8_t linefollow_pid_valid;

void LineFollow_ResetLinePID(void)
{
    linefollow_pid_integral = 0.0f;
    linefollow_pid_last_error = 0;
    linefollow_pid_valid = 0;
}

/* state为八路灰度权重和，0=居中。返回左右差速值。 */
int LineFollow_ComputeLinePID(int error)
{
    float derivative = 0.0f;
    int output;

    if (linefollow_pid_valid)
        derivative = (float)(error - linefollow_pid_last_error);
    linefollow_pid_last_error = error;
    linefollow_pid_valid = 1;

    linefollow_pid_integral += error;
    if (linefollow_pid_integral > LINE_FOLLOW_I_LIMIT)
        linefollow_pid_integral = LINE_FOLLOW_I_LIMIT;
    if (linefollow_pid_integral < -LINE_FOLLOW_I_LIMIT)
        linefollow_pid_integral = -LINE_FOLLOW_I_LIMIT;

    output = (int)(LINE_FOLLOW_KP * error +
                   LINE_FOLLOW_KI * linefollow_pid_integral +
                   LINE_FOLLOW_KD * derivative);

    if (output > LINE_FOLLOW_TURN_LIMIT) output = LINE_FOLLOW_TURN_LIMIT;
    if (output < -LINE_FOLLOW_TURN_LIMIT) output = -LINE_FOLLOW_TURN_LIMIT;

    return output;
}

/* 主循环命令以单次四轮事务发布，200Hz ISR不会读到半更新的目标向量。 */
void LineFollow_SetWheelTargets(int m1, int m2, int m3, int m4)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    wheel_target[1] = m1;
    wheel_target[2] = m2;
    wheel_target[3] = m3;
    wheel_target[4] = m4;
    if (primask == 0U) __enable_irq();
}

void LineFollow_Stop(void)
{
    LineFollow_SetWheelTargets(0, 0, 0, 0);
}

void LineFollow_Init(void)
{
    ChassisPos_ResetVelocityPid();
    HWT101_UART_Init();
    HeadingHold_Init();
    ChassisPos_Init();
}
