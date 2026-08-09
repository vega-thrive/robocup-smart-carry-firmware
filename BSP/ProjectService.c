#include "ProjectService.h"
#include "LineFollow.h"
#include "HWT101.h"
#include "Sensor.h"
#include "ChassisMotor.h"
#include "CompetitionQr.h"
#include "Servo4.h"
#include "VisionAlignment.h"
#include "MaxiCam.h"
#include <stdio.h>

/* 任务1路段测试: 1 = 完整跑任务1路段(出发→循迹→对准→中段运动),
 * 到达任务2扫码点(FINAL_LEFT_40 第一次左移完成)自动停车, 不进入任务2.
 * 0 = 完整比赛流程(含任务2奖杯抓取). */
#define TASK1_ROUTE_TEST_MODE 0

/* ---- 状态变量 ---- */
static volatile uint8_t task1_route_test_done;
static volatile RouteState route_state = ROUTE_HOME_LAUNCH;
static volatile RouteState pause_next_mode;
static volatile RouteState vision_next_mode;
static volatile uint8_t vision_started;
static volatile int marker_suppress_ticks;
static volatile int home_launch_tick;
static volatile int mode_entry_tick;
static volatile uint16_t home_fixed_ticks;
static uint8_t trophy_phase;
static uint16_t trophy_ticks;
static uint8_t trophy_cam_mode_on;
static uint8_t trophy_det_confirm;
static uint32_t trophy_det_last_seq;
static uint8_t trophy_det_count;
static uint32_t trophy_det_gap_until_ms;
static uint8_t final_left_stage;
static uint8_t final_left_heading_stable_ticks;
static float left_heading_absolute;
static uint8_t left_heading_valid;
static int8_t yaw_ccw_sign = 1;
static float home_turn_start_absolute;
static int last_steer;
static int cross_marker_ticks;
static uint8_t lcd_div;
static int debug_line_state;
static int debug_turn_target;

/* ---- 对外状态查询 ---- */
int ProjectService_GetRouteState(void)
{
    return (int)route_state;
}

/* 模式2(0x02)奖杯检测: 连续 TROPHY_DETECT_CONFIRM 帧 code=26 算命中一次,
 * 每次命中计入一个奖杯, 然后进入 TROPHY_DETECT_CLEAR_MS 清屏期, 防止
 * 同一奖杯还在 ROI 内被连续重复计数.
 * 左移未越过 TROPHY_DETECT_ARM_COUNTS 时忽略, 防止刚夹住的奖杯仍在
 * 摄像头下半屏 ROI 内, 新的一轮刚启动就误触发. */
static uint8_t TrophyDetect_Seen(void)
{
    MaxiCam_Frame frame;

    if (ChassisPos_GetLeftPosition() < (int)TROPHY_DETECT_ARM_COUNTS)
        return 0U;

    if (!MaxiCam_GetLatest(&frame, 250U))
        return 0U;
    if (frame.sequence == trophy_det_last_seq)
        return 0U;
    trophy_det_last_seq = frame.sequence;

    if ((int32_t)(HAL_GetTick() - trophy_det_gap_until_ms) < 0)
        return 0U;

    if (frame.data[0] != TROPHY_DETECT_CODE)
    {
        trophy_det_confirm = 0U;
        return 0U;
    }
    if (++trophy_det_confirm >= TROPHY_DETECT_CONFIRM)
    {
        trophy_det_confirm = 0U;
        trophy_det_gap_until_ms = HAL_GetTick() + TROPHY_DETECT_CLEAR_MS;
        return 1U;
    }
    return 0U;
}

/* ---- 初始化 ---- */
void ProjectService_Init(void)
{
    LineFollow_Init();
    HWT101_getAngle_Reset();
    LineFollow_ResetLinePID();
    last_steer = 0;
    task1_route_test_done = 0U;
    Servo4_UseTask1Parameters();
    /* 上电先退掉摄像头可能残留的模式循环(0x04), 再启动颜色识别 */
    (void)MaxiCam_SetMode(MAXICAM_MODE_STOP);
    Task1Color_Start();
    ChassisPos_DrawStatus(1, "Home Launch");
    HAL_TIM_Base_Start_IT(&htim7);
}

/* ---- 主循环 ---- */
void ProjectService_Run(void)
{
    char buf[24];

    ChassisPos_FlushDisplay();
    Task1Color_ServiceCommands();
    Task1Color_Process();

    if (task1_route_test_done)
    {
        if (++lcd_div >= 5)
        {
            ChassisPos_DrawStatus(20, "T1 ROUTE DONE");
            lcd_div = 0;
        }
        return;
    }

    if (route_state == ROUTE_VISION_ALIGN)
    {
        VisionAlignment_Status vision_status;

        if (!vision_started)
        {
            VisionAlignment_Start();
            vision_started = 1U;
            ChassisPos_DrawStatus(1, "Vision Align");
        }

        vision_status = VisionAlignment_GetStatus();
        if ((vision_status == VISION_ALIGN_DONE) ||
            (vision_status == VISION_ALIGN_TIMEOUT))
        {
            VisionAlignment_Cancel();
            ChassisPos_ResetEncoder();
            ChassisPos_ResetVelocityPid();
            HWT101_getAngle_Reset();
            HeadingHold_Reset();
            LineFollow_Stop();
            if (vision_next_mode == ROUTE_REAR_ALIGN)
            {
                ChassisPos_Start(-46250, 46250, -46250, 46250);
                ChassisPos_DrawStatus(1, "Left 25cm");
            }
            else if (vision_next_mode == ROUTE_REAR_ALIGN_2)
            {
                ChassisPos_Start(-54964, -54964, -54964, -54964);
                ChassisPos_DrawStatus(1, "Back 30cm");
            }
            else if (vision_next_mode == ROUTE_FINAL_LEFT_FORWARD)
            {
                ChassisPos_Start(-55500, -55500, -55500, -55500);
                ChassisPos_DrawStatus(1, "Back 30cm");
            }
            __disable_irq();
            route_state = vision_next_mode;
            __enable_irq();
            mode_entry_tick = 0;
            lcd_div = 0;
            vision_started = 0U;
            ChassisPos_DrawStatus(1, "Route Resume");
        }
        return;
    }

    if (route_state == ROUTE_HOME_LAUNCH)
    {
        if (++lcd_div >= 5)
        {
            if (home_launch_tick < HOME_LAUNCH_FWD_TICKS)
                sprintf(buf, "Launch T:%3d", home_launch_tick);
            else
                sprintf(buf, "Turn A:%3d", (int)Angle);
            ChassisPos_DrawStatus(20, buf);
            lcd_div = 0;
        }
        return;
    }

    if (route_state == ROUTE_HOME_FIXED_FORWARD)
    {
        if (++lcd_div >= 5)
        {
            char buf[24];
            sprintf(buf, "T:%3d S:%3d P:%5d",
                    wheel_target[1], debug_trap_speed, debug_trap_position);
            ChassisPos_DrawStatus(20, buf);
            sprintf(buf, "E:%5d F:%1u A:%1u",
                    debug_trap_error, pos_fault, pos_control_active);
            ChassisPos_DrawStatus(40, buf);
            lcd_div = 0;
        }
        return;
    }

    if (route_state == ROUTE_HOME_FIXED_PAUSE)
    {
        if (++lcd_div >= 5)
        {
            CompetitionQrStatus qr = CompetitionQr_GetStatus();
            if (qr == COMP_QR_READY)
                sprintf(buf, "T1 QR:%u", CompetitionQr_GetTaskCode(COMP_QR_TASK1));
            else if (qr == COMP_QR_DEFAULTED)
                sprintf(buf, "T1 DEFAULT:1");
            else
                sprintf(buf, "T1 SCAN:%u", CompetitionQr_GetAttemptCount());
            ChassisPos_DrawStatus(20, buf);
            lcd_div = 0;
        }
        return;
    }

    if (route_state == ROUTE_HOME_FIXED_LEFT)
    {
        if (++lcd_div >= 5)
        {
            ChassisPos_DrawStatus(20, "Home Left");
            lcd_div = 0;
        }
        return;
    }

    if (route_state == ROUTE_HOME_FIXED_SETTLE)
    {
        if (++lcd_div >= 5)
        {
            sprintf(buf, "Settle:%2u", home_fixed_ticks);
            ChassisPos_DrawStatus(20, buf);
            lcd_div = 0;
        }
        return;
    }

    if (route_state == ROUTE_ROTATE_90)
    {
        if (++lcd_div >= 5)
        {
            sprintf(buf, "Rot90 A:%3d", (int)Angle);
            ChassisPos_DrawStatus(20, buf);
            lcd_div = 0;
        }
        return;
    }

    if (route_state == ROUTE_CURVE)
    {
        if (++lcd_div >= 5)
        {
            sprintf(buf, "Ph:%1d D:%5ld",
                    mode_entry_tick,
                    (long)((abs(encoder[1]) + abs(encoder[2])
                          + abs(encoder[3]) + abs(encoder[4])) / 4));
            ChassisPos_DrawStatus(20, buf);
            lcd_div = 0;
        }
        return;
    }

    if (route_state == ROUTE_REAR_ALIGN)
    {
        if (++lcd_div >= 5)
        {
            sprintf(buf, "Ph:%1d D:%5ld", mode_entry_tick,
                    (long)((abs(encoder[1]) + abs(encoder[2])
                          + abs(encoder[3]) + abs(encoder[4])) / 4));
            ChassisPos_DrawStatus(20, buf);
            lcd_div = 0;
        }
        return;
    }

    /* if (route_state == ROUTE_FINAL_FORWARD_1)  --  replaced by REAR_ALIGN phase 1 */

    if (route_state == ROUTE_PAUSE)
    {
        if (++lcd_div >= 5)
        {
            sprintf(buf, "Pause:%3d", mode_entry_tick);
            ChassisPos_DrawStatus(20, buf);
            lcd_div = 0;
        }
        return;
    }

    if (route_state == ROUTE_REAR_ALIGN_2)
    {
        if (++lcd_div >= 5)
        {
            int total_distance = (abs(encoder[1]) + abs(encoder[2])
                               + abs(encoder[3]) + abs(encoder[4])) / 4;
            sprintf(buf, "Rear2 D:%5d X:%1d", total_distance, mode_entry_tick);
            ChassisPos_DrawStatus(20, buf);
            lcd_div = 0;
        }
        return;
    }

    if (route_state == ROUTE_PAUSE_2)
    {
        if (++lcd_div >= 5)
        {
            sprintf(buf, "Pause2:%3d", mode_entry_tick);
            ChassisPos_DrawStatus(20, buf);
            lcd_div = 0;
        }
        return;
    }

    if (route_state == ROUTE_FINAL_LEFT_FORWARD)
    {
        if (++lcd_div >= 5)
        {
            int dist = (abs(encoder[1]) + abs(encoder[2])
                     + abs(encoder[3]) + abs(encoder[4])) / 4;
            sprintf(buf, "LF Ph:%1d D:%5d", mode_entry_tick, dist);
            ChassisPos_DrawStatus(20, buf);
            lcd_div = 0;
        }
        return;
    }

    if (route_state == ROUTE_FINAL_ROTATE_69)
    {
        if (++lcd_div >= 5)
        {
            sprintf(buf, "Rot69 A:%3d", (int)Angle);
            ChassisPos_DrawStatus(20, buf);
            lcd_div = 0;
        }
        return;
    }

    if (route_state == ROUTE_FINAL_FORWARD_2)
    {
        if (++lcd_div >= 5)
        {
            int fwd = (encoder[1] + encoder[2] + encoder[3] + encoder[4]) / 4;
            sprintf(buf, "Fwd2:%5ld", (long)fwd);
            ChassisPos_DrawStatus(20, buf);
            lcd_div = 0;
        }
        return;
    }

    if (route_state == ROUTE_FINAL_ROTATE_30)
    {
        if (++lcd_div >= 5)
        {
            sprintf(buf, "Rot30 A:%3d", (int)Angle);
            ChassisPos_DrawStatus(20, buf);
            lcd_div = 0;
        }
        return;
    }

    if (route_state == ROUTE_FINAL_LEFT_40)
    {
        if (++lcd_div >= 5)
        {
            ChassisPos_DrawStatus(20, "Left 20");
            lcd_div = 0;
        }
        return;
    }

    if (route_state == ROUTE_FINAL_ROTATE_180)
    {
        if (++lcd_div >= 5)
        {
            sprintf(buf, "Rot180 A:%3d", (int)Angle);
            ChassisPos_DrawStatus(20, buf);
            lcd_div = 0;
        }
        return;
    }

    if (route_state == ROUTE_PAUSE_GENERIC)
    {
        if (++lcd_div >= 5)
        {
            if ((pause_next_mode == ROUTE_FINAL_LEFT_40) &&
                (final_left_stage == 1U))
            {
                CompetitionQrStatus qr = CompetitionQr_GetStatus();
                if (qr == COMP_QR_READY)
                    sprintf(buf, "T2 QR:%u",
                            CompetitionQr_GetTaskCode(COMP_QR_TASK2));
                else if (qr == COMP_QR_DEFAULTED)
                    sprintf(buf, "T2 DEFAULT:1");
                else
                    sprintf(buf, "T2 SCAN:%u",
                            CompetitionQr_GetAttemptCount());
            }
            else
            {
                sprintf(buf, "Pause:%3d", mode_entry_tick);
            }
            ChassisPos_DrawStatus(20, buf);
            lcd_div = 0;
        }
        return;
    }

    /* 最终179°旋转后的返程循迹模式（控制已移至ISR，此处仅显示） */
    if (route_state == ROUTE_TRACK_RETURN)
    {
        if (++lcd_div >= 10)
        {
            sprintf(buf, "R S:%2d T:%3d", debug_line_state, debug_turn_target);
            ChassisPos_DrawStatus(20, buf);
            sprintf(buf, "W:%3d V:%3d", wheel_target[1], velocity[1]);
            ChassisPos_DrawStatus(40, buf);
            lcd_div = 0;
        }
        return;
    }

    if (route_state == ROUTE_TRACK_FIRST)
    {
        if (++lcd_div >= 10)
        {
            sprintf(buf, "S:%2d T:%3d", debug_line_state, debug_turn_target);
            ChassisPos_DrawStatus(20, buf);
            sprintf(buf, "W:%3d V:%3d", wheel_target[1], velocity[1]);
            ChassisPos_DrawStatus(40, buf);
            lcd_div = 0;
        }
        return;
    }
}

/* ---- 200Hz ISR ---- */
void ProjectService_OnTimerElapsed(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM7)
    {
        return;
    }

    ChassisMotor_SampleEncoders();

    if (task1_route_test_done)
    {
        LineFollow_Stop();
        ChassisMotor_ControlTick(wheel_target);
        return;
    }

    if (route_state == ROUTE_HOME_LAUNCH)
    {
        if (home_launch_tick == 0)
        {
            /* First tick: start TrapMove position control for forward burst. */
            ChassisPos_Start(120000, 120000, 120000, 120000);
            home_launch_tick = 1;
        }

        if (home_launch_tick == 1)
        {
            if (ChassisPos_Update(HOME_LAUNCH_SPEED, 0, 0))
            {
                /* Forward burst complete, begin rotation. */
                ChassisPos_ResetVelocityPid();
                HWT101_getAngle_Reset();
                HeadingHold_Reset();
                home_turn_start_absolute = HWT101_GetAbsoluteYaw();
                home_launch_tick = 2;
            }
            /* else: still moving forward, UpdatePositionControl has set
             * wheel_target. Fall through to ChassisMotor_ControlTick below. */
        }

        if (home_launch_tick >= 2)
        {
            float a;
            int spd = HOME_TURN_SPEED;
            a = Angle;
            if (a < 0.0f) a = -a;
            if (a > 70.0f)
            {
                spd = HOME_TURN_SPEED - (int)((a - 70.0f) * 13.2f);
                if (spd < 120) spd = 120;
            }
            wheel_target[1] = -spd;
            wheel_target[2] = -spd;
            wheel_target[3] =  spd;
            wheel_target[4] =  spd;
            if (a >= HOME_TURN_ANGLE)
            {
                float ccw_delta = HeadingHold_WrapAngle180(
                    HWT101_GetAbsoluteYaw() - home_turn_start_absolute);
                yaw_ccw_sign = (ccw_delta >= 0.0f) ? 1 : -1;
                left_heading_absolute = HWT101_GetAbsoluteYaw();
                left_heading_valid = 1;
                ChassisPos_ResetVelocityPid();
                HWT101_getAngle_Reset();
                HeadingHold_Reset();
                home_fixed_ticks = 0;
                route_state = ROUTE_HOME_FIXED_FORWARD;
                lcd_div = 0;
                mode_entry_tick = 0;
                ChassisPos_Start(155400, 155400, 155400, 155400);
                ChassisPos_DrawStatus(1, "Home Fwd 20cm");
            }
        }
    }

    if (route_state == ROUTE_HOME_FIXED_FORWARD)
    {
        if (ChassisPos_Update(HOME_FIXED_FORWARD_SPEED, 0,
              HeadingHold_GetFreshCorrection()))
        {
            LineFollow_Stop();
            ChassisPos_ResetVelocityPid();
            home_fixed_ticks = 0;
            route_state = ROUTE_HOME_FIXED_PAUSE;
            lcd_div = 0;
            CompetitionQr_RequestStart(COMP_QR_TASK1);
            Task1Color_Start();
            ChassisPos_DrawStatus(1, "Scan Task 1");
        }
    }

    if (route_state == ROUTE_HOME_FIXED_PAUSE)
    {
        LineFollow_Stop();
        CompetitionQrStatus qr_status = CompetitionQr_GetStatus();
        home_fixed_ticks++;
        if ((home_fixed_ticks >= HOME_FIXED_PAUSE_TICKS) &&
            ((qr_status == COMP_QR_READY) ||
             (qr_status == COMP_QR_DEFAULTED)))
        {
            ChassisPos_ResetVelocityPid();
            HeadingHold_Reset();
            home_fixed_ticks = 0;
            lcd_div = 0;
            route_state = ROUTE_HOME_FIXED_LEFT;
            ChassisPos_Start(-(int)HOME_FIXED_LEFTBACK_BACK_DIST,
                             -(int)HOME_FIXED_LEFTBACK_BACK_DIST,
                             -(int)HOME_FIXED_LEFTBACK_BACK_DIST,
                             -(int)HOME_FIXED_LEFTBACK_BACK_DIST);
            ChassisPos_DrawStatus(1, "Home Back");
        }
    }

    if (route_state == ROUTE_HOME_FIXED_LEFT)
    {
        if (home_fixed_ticks == 0)
        {
            if (ChassisPos_Update(HOME_FIXED_LEFTBACK_FWD_SPEED, 0,
                  HeadingHold_GetFreshCorrection()))
            {
                ChassisPos_ResetVelocityPid();
                home_fixed_ticks = 1;
                ChassisPos_Start(-(int)HOME_FIXED_LEFTBACK_LEFT_DIST,
                                  (int)HOME_FIXED_LEFTBACK_LEFT_DIST,
                                 -(int)HOME_FIXED_LEFTBACK_LEFT_DIST,
                                  (int)HOME_FIXED_LEFTBACK_LEFT_DIST);
                ChassisPos_DrawStatus(1, "Home Left");
            }
        }
        else
        {
            uint8_t pos_done = ChassisPos_Update(0,
                  HOME_FIXED_LEFTBACK_LEFT_SPEED,
                  HeadingHold_GetFreshCorrection());
            Sensor_Get();
            {
                int bc = GS_state[1] + GS_state[2] + GS_state[3] + GS_state[4]
                        + GS_state[5] + GS_state[6] + GS_state[7] + GS_state[8];
                if (bc == 2 && GS_state[4] && GS_state[5])
                    mode_entry_tick++;
                else
                    mode_entry_tick = 0;
            }
            if (pos_done || mode_entry_tick >= 3)
            {
                LineFollow_Stop();
                ChassisPos_ResetVelocityPid();
                home_fixed_ticks = 0;
                route_state = ROUTE_HOME_FIXED_SETTLE;
                lcd_div = 0;
                mode_entry_tick = 0;
                ChassisPos_DrawStatus(1, "Home Settle");
            }
        }
    }

    if (route_state == ROUTE_HOME_FIXED_SETTLE)
    {
        LineFollow_Stop();
        if (++home_fixed_ticks >= HOME_FIXED_SETTLE_TICKS)
        {
            ChassisPos_ResetVelocityPid();
            HeadingHold_Reset();
            last_steer = 0;
            LineFollow_ResetLinePID();
            marker_suppress_ticks = 0;
            mode_entry_tick = 0;
            lcd_div = 0;
            route_state = ROUTE_TRACK_FIRST;
            Task1Color_Start();
            ChassisPos_DrawStatus(1, "Line Follow");
        }
    }

    if (route_state == ROUTE_TRACK_RETURN)
    {
        if (marker_suppress_ticks < MARKER_SUPPRESS_TICKS)
        {
            marker_suppress_ticks++;
        }
    }

    if (route_state == ROUTE_ROTATE_90)
    {
        float a = Angle;
        if (a < 0.0f) a = -a;
        if (a >= ROTATE_90_TARGET_ANGLE)
        {
            ChassisPos_ResetEncoder();
            ChassisPos_ResetVelocityPid();
            HWT101_getAngle_Reset();
            HeadingHold_Reset();
            ChassisPos_Start(CURVE_ENCODER_COUNTS, CURVE_ENCODER_COUNTS,
                            CURVE_ENCODER_COUNTS, CURVE_ENCODER_COUNTS);
            route_state = ROUTE_CURVE;
            mode_entry_tick = 0;
            lcd_div = 0;
        }
        else
        {
            int spd = ROTATE_90_SPEED;
            if (a > 70.0f)
            {
                spd = 372 - (int)((a - 70.0f) * 13.6f);
                if (spd < 80) spd = 80;
            }
            wheel_target[1] =  spd;
            wheel_target[2] =  spd;
            wheel_target[3] = -spd;
            wheel_target[4] = -spd;
        }
    }

    if (route_state == ROUTE_CURVE)
    {
        if (mode_entry_tick == 0)
        {
            int cross_marker;
            Sensor_Get();
            {
                int bc = GS_state[1] + GS_state[2] + GS_state[3] + GS_state[4]
                        + GS_state[5] + GS_state[6] + GS_state[7] + GS_state[8];
                cross_marker = (bc >= 3) && (bc <= 4)
                            && (GS_state[3] || GS_state[4]);
            }
            if (cross_marker)
                cross_marker_ticks++;
            else
                cross_marker_ticks = 0;

            if (ChassisPos_Update(CURVE_FORWARD_SPEED, 0,
                  HeadingHold_GetFreshCorrection())
                || cross_marker_ticks >= 3)
            {
                ChassisPos_ResetVelocityPid();
                mode_entry_tick = 1;
                cross_marker_ticks = 0;
                {
                    int right = (int)((long)CURVE_ENCODER_COUNTS
                                    * CURVE_RIGHT_SPEED / CURVE_FORWARD_SPEED);
                    ChassisPos_Start(right, -right, -right, right);
                }
                ChassisPos_DrawStatus(1, "Right");
            }
        }
        else
        {
            if (ChassisPos_Update(0, CURVE_RIGHT_SPEED,
                  HeadingHold_GetFreshCorrection()))
            {
                ChassisPos_ResetVelocityPid();
                LineFollow_Stop();
                vision_next_mode = ROUTE_REAR_ALIGN;
                vision_started = 0U;
                pause_next_mode = ROUTE_VISION_ALIGN;
                route_state = ROUTE_PAUSE_GENERIC;
                mode_entry_tick = 0;
                lcd_div = 0;
                ChassisPos_DrawStatus(1, "Pause Vision");
                Move_Control(0, 0, 0, 0);
                return;
            }
        }
    }

    if (route_state == ROUTE_REAR_ALIGN)
    {
        if (mode_entry_tick == 0)
        {
            if (ChassisPos_Update(0, 300,
                  HeadingHold_GetFreshCorrection()))
            {
                ChassisPos_ResetVelocityPid();
                mode_entry_tick = 1;
                ChassisPos_Start(74000, 74000, 74000, 74000);
                ChassisPos_DrawStatus(1, "Fwd 40cm");
            }
        }
        else
        {
            if (ChassisPos_Update(FINAL_FORWARD_SPEED, 0,
                  HeadingHold_GetFreshCorrection()))
            {
                ChassisPos_ResetVelocityPid();
                LineFollow_Stop();
                route_state = ROUTE_PAUSE;
                mode_entry_tick = 0;
                lcd_div = 0;
                ChassisPos_DrawStatus(1, "Pause");
                Move_Control(0, 0, 0, 0);
                return;
            }
        }
    }

    /* FINAL_FORWARD_1 removed -- replaced by REAR_ALIGN phase 1 */

    if (route_state == ROUTE_PAUSE)
    {
        mode_entry_tick++;
        if (mode_entry_tick >= PAUSE_TICKS)
        {
            ChassisPos_ResetVelocityPid();
            HWT101_getAngle_Reset();
            HeadingHold_Reset();
            vision_next_mode = ROUTE_REAR_ALIGN_2;
            vision_started = 0U;
            route_state = ROUTE_VISION_ALIGN;
            mode_entry_tick = 0;
            lcd_div = 0;
            ChassisPos_DrawStatus(1, "Vision Align");
        }
        else
        {
            LineFollow_Stop();
        }
    }

    if (route_state == ROUTE_REAR_ALIGN_2)
    {
        if (mode_entry_tick == 0)
        {
            if (ChassisPos_Update(REAR_ALIGN_2_BACK_SPEED, 0,
                  HeadingHold_GetFreshCorrection()))
            {
                ChassisPos_ResetVelocityPid();
                mode_entry_tick = 1;
                ChassisPos_Start(-REAR_ALIGN_2_DISTANCE,  REAR_ALIGN_2_DISTANCE,
                                -REAR_ALIGN_2_DISTANCE,  REAR_ALIGN_2_DISTANCE);
                ChassisPos_DrawStatus(1, "Left 85cm");
            }
        }
        else
        {
            int cross_marker;
            Sensor_Get();
            {
                int bc = GS_state[1] + GS_state[2] + GS_state[3] + GS_state[4]
                        + GS_state[5] + GS_state[6] + GS_state[7] + GS_state[8];
                cross_marker = (bc >= 3)
                            && (GS_state[3] || GS_state[4]);
            }
            if (cross_marker)
                cross_marker_ticks++;
            else
                cross_marker_ticks = 0;

            if (ChassisPos_Update(0, REAR_ALIGN_2_LEFT_SPEED,
                  HeadingHold_GetFreshCorrection())
                || cross_marker_ticks >= 3)
            {
                ChassisPos_ResetVelocityPid();
                LineFollow_Stop();
                route_state = ROUTE_PAUSE_2;
                mode_entry_tick = 0;
                cross_marker_ticks = 0;
                lcd_div = 0;
                ChassisPos_DrawStatus(1, "Pause 2");
                Move_Control(0, 0, 0, 0);
                return;
            }
        }
    }

    if (route_state == ROUTE_PAUSE_2)
    {
        mode_entry_tick++;
        if (mode_entry_tick >= PAUSE_2_TICKS)
        {
            ChassisPos_ResetEncoder();
            ChassisPos_ResetVelocityPid();
            HWT101_getAngle_Reset();
            HeadingHold_Reset();
            vision_next_mode = ROUTE_FINAL_LEFT_FORWARD;
            vision_started = 0U;
            route_state = ROUTE_VISION_ALIGN;
            mode_entry_tick = 0;
            lcd_div = 0;
            ChassisPos_DrawStatus(1, "Vision Align");
        }
        else
        {
            LineFollow_Stop();
        }
    }

    if (route_state == ROUTE_FINAL_LEFT_FORWARD)
    {
        if (mode_entry_tick == 0)
        {
            if (ChassisPos_Update(FINAL_LF_FORWARD_SPEED, 0,
                  HeadingHold_GetFreshCorrection()))
            {
                ChassisPos_ResetVelocityPid();
                mode_entry_tick = 1;
                {
                    int lat = (int)((long)FINAL_LF_DISTANCE
                                  * FINAL_LF_LEFT_SPEED / FINAL_LF_FORWARD_SPEED);
                    lat = lat * 2 - 74000;
                    ChassisPos_Start(-lat, lat, -lat, lat);
                }
                ChassisPos_DrawStatus(1, "Left 72cm");
            }
        }
        else if (mode_entry_tick == 1)
        {
            if (ChassisPos_Update(0, FINAL_LF_LEFT_SPEED,
                  HeadingHold_GetFreshCorrection()))
            {
                ChassisPos_ResetVelocityPid();
                mode_entry_tick = 2;
                ChassisPos_Start(111000, 111000, 111000, 111000);
                ChassisPos_DrawStatus(1, "Fwd 60cm");
            }
        }
        else
        {
            if (ChassisPos_Update(FINAL_LF_FORWARD_SPEED, 0,
                  HeadingHold_GetFreshCorrection()))
            {
                LineFollow_Stop();
                vision_next_mode = ROUTE_FINAL_ROTATE_69;
                vision_started = 0U;
                pause_next_mode = ROUTE_VISION_ALIGN;
                route_state = ROUTE_PAUSE_GENERIC;
                mode_entry_tick = 0;
                lcd_div = 0;
                ChassisPos_DrawStatus(1, "Pause");
                Move_Control(0, 0, 0, 0);
                return;
            }
        }
    }

    if (route_state == ROUTE_FINAL_ROTATE_69)
    {
        float a = Angle;
        if (a < 0.0f) a = -a;
        if (a >= FINAL_ROTATE_69_ANGLE)
        {
            LineFollow_Stop();
            pause_next_mode = ROUTE_FINAL_FORWARD_2;
            route_state = ROUTE_PAUSE_GENERIC;
            mode_entry_tick = 0;
            lcd_div = 0;
            ChassisPos_DrawStatus(1, "Pause");
        }
        else
        {
            wheel_target[1] =  FINAL_ROTATE_69_SPEED;
            wheel_target[2] =  FINAL_ROTATE_69_SPEED;
            wheel_target[3] = -FINAL_ROTATE_69_SPEED;
            wheel_target[4] = -FINAL_ROTATE_69_SPEED;
        }
    }

    if (route_state == ROUTE_FINAL_FORWARD_2)
    {
        if (ChassisPos_Update(FINAL_FORWARD_2_SPEED, 0,
              HeadingHold_GetFreshCorrection()))
        {
            LineFollow_Stop();
            vision_next_mode = ROUTE_FINAL_ROTATE_30;
            vision_started = 0U;
            pause_next_mode = ROUTE_VISION_ALIGN;
            route_state = ROUTE_PAUSE_GENERIC;
            mode_entry_tick = 0;
            lcd_div = 0;
            ChassisPos_DrawStatus(1, "Pause");
            Move_Control(0, 0, 0, 0);
            return;
        }
    }

    if (route_state == ROUTE_FINAL_ROTATE_30)
    {
        float error_ccw;
        float error_abs;
        int rotate_speed;

        if (!left_heading_valid || !HWT101_IsReady())
        {
            LineFollow_Stop();
            final_left_heading_stable_ticks = 0;
        }
        else
        {
            error_ccw = HeadingHold_WrapAngle180(
                left_heading_absolute - HWT101_GetAbsoluteYaw()) * yaw_ccw_sign;
            error_abs = (error_ccw >= 0.0f) ? error_ccw : -error_ccw;

            if (error_abs <= 1.5f)
            {
                LineFollow_Stop();
                if (++final_left_heading_stable_ticks >= 10)
                {
                    LineFollow_Stop();
                    pause_next_mode = ROUTE_FINAL_LEFT_40;
                    route_state = ROUTE_PAUSE_GENERIC;
                    mode_entry_tick = 0;
                    lcd_div = 0;
                    final_left_heading_stable_ticks = 0;
                    final_left_stage = 0;
                    ChassisPos_DrawStatus(1, "Pause");
                    Move_Control(0, 0, 0, 0);
                    return;
                }
            }
            else
            {
                final_left_heading_stable_ticks = 0;

                if (error_abs > 15.0f)      rotate_speed = FINAL_ROTATE_30_SPEED;
                else if (error_abs > 6.0f)  rotate_speed = 264;
                else                        rotate_speed = 168;

                if (error_ccw > 0.0f)
                {
                    wheel_target[1] = -rotate_speed;
                    wheel_target[2] = -rotate_speed;
                    wheel_target[3] =  rotate_speed;
                    wheel_target[4] =  rotate_speed;
                }
                else
                {
                    wheel_target[1] =  rotate_speed;
                    wheel_target[2] =  rotate_speed;
                    wheel_target[3] = -rotate_speed;
                    wheel_target[4] = -rotate_speed;
                }
            }
        }
    }

    if (route_state == ROUTE_FINAL_LEFT_40)
    {
        if (ChassisPos_Update(0, FINAL_LEFT_40_SPEED,
              HeadingHold_GetFreshCorrection()))
        {
            LineFollow_Stop();
#if TASK1_ROUTE_TEST_MODE
            /* 任务1路段测试: 到达任务2扫码点(第一次左移完成)即停车. */
            ChassisPos_ResetVelocityPid();
            Move_Control(0, 0, 0, 0);
            task1_route_test_done = 1U;
            ChassisPos_DrawStatus(1, "T1 ROUTE DONE");
            return;
#else
            if (final_left_stage == 0)
            {
                final_left_stage = 1;
                pause_next_mode = ROUTE_FINAL_LEFT_40;
                CompetitionQr_RequestStart(COMP_QR_TASK2);
            }
            else
            {
                pause_next_mode = ROUTE_FINAL_ROTATE_180;
            }
            route_state = ROUTE_PAUSE_GENERIC;
            mode_entry_tick = 0;
            lcd_div = 0;
            ChassisPos_DrawStatus(1, "Pause");
            Move_Control(0, 0, 0, 0);
            return;
#endif
        }
    }

    if (route_state == ROUTE_PAUSE_GENERIC)
    {
        uint8_t waiting_for_task2 =
            ((pause_next_mode == ROUTE_FINAL_LEFT_40) &&
             (final_left_stage == 1U));
        CompetitionQrStatus qr_status = CompetitionQr_GetStatus();
        mode_entry_tick++;
        if ((mode_entry_tick >= PAUSE_GENERIC_TICKS) &&
            ((!waiting_for_task2) ||
             (qr_status == COMP_QR_READY) ||
             (qr_status == COMP_QR_DEFAULTED)))
        {
            ChassisPos_ResetVelocityPid();
            HWT101_getAngle_Reset();
            HeadingHold_Reset();
            switch (pause_next_mode)
            {
                case ROUTE_FINAL_ROTATE_69:
                    ChassisPos_ResetEncoder();
                    ChassisPos_DrawStatus(1, "Rotate 55 CW"); break;
                case ROUTE_FINAL_FORWARD_2:
                    ChassisPos_Start(55500, 55500, 55500, 55500);
                    ChassisPos_DrawStatus(1, "Final Fwd 2"); break;
                case ROUTE_FINAL_ROTATE_30:
                    ChassisPos_ResetEncoder();
                    ChassisPos_DrawStatus(1, "Rotate 30 CW"); break;
                case ROUTE_FINAL_LEFT_40:
                    if (final_left_stage == 0)
                        ChassisPos_Start(
                            -203500,  203500, -203500,  203500);
                    else
                        ChassisPos_Start(
                            -37000,  37000, -37000,  37000);
                    ChassisPos_DrawStatus(1, "Left 20 cm"); break;
                case ROUTE_FINAL_ROTATE_180:
                    ChassisPos_ResetEncoder();
                    ChassisPos_DrawStatus(1, "Rotate 180 CW"); break;
                case ROUTE_VISION_ALIGN:
                    ChassisPos_DrawStatus(1, "Vision Align"); break;
                default: break;
            }
            route_state = pause_next_mode;
            mode_entry_tick = 0;
            lcd_div = 0;
        }
        else
        {
            LineFollow_Stop();
        }
    }

    if (route_state == ROUTE_FINAL_ROTATE_180)
    {
        float a = Angle;
        if (a < 0.0f) a = -a;
        if (a >= FINAL_ROTATE_180_ANGLE)
        {
            ChassisPos_ResetVelocityPid();
            HeadingHold_Reset();
            LineFollow_Stop();
            route_state = ROUTE_TRACK_RETURN;
            LineFollow_ResetLinePID();
            marker_suppress_ticks = 0;
            lcd_div = 0;
            mode_entry_tick = 0;
            ChassisPos_DrawStatus(1, "Line Follow");
        }
        else
        {
            wheel_target[1] =  FINAL_ROTATE_180_SPEED;
            wheel_target[2] =  FINAL_ROTATE_180_SPEED;
            wheel_target[3] = -FINAL_ROTATE_180_SPEED;
            wheel_target[4] = -FINAL_ROTATE_180_SPEED;
        }
    }

    if (route_state == ROUTE_TROPHY)
    {
        int hc;

        if (trophy_phase == 0 || trophy_phase == 4
            || trophy_phase == 6 || trophy_phase == 8 || trophy_phase == 10)
        {
            LineFollow_Stop();
            if (trophy_phase != 10 && ++trophy_ticks >= TROPHY_PAUSE_TICKS)
            {
                ChassisPos_ResetVelocityPid();
                ChassisPos_ResetEncoder();
                HWT101_getAngle_Reset();
                HeadingHold_Reset();
                trophy_ticks = 0;
                trophy_phase++;
                if ((trophy_phase == 5U) ||
                    (trophy_phase == 7U) ||
                    (trophy_phase == 9U))
                {
                    vision_next_mode = ROUTE_TROPHY;
                    vision_started = 0U;
                    route_state = ROUTE_VISION_ALIGN;
                    ChassisPos_DrawStatus(1, "Vision Align");
                }
            }
        }
        else if (trophy_phase == 1)
        {
            float a = Angle;
            if (a < 0.0f) a = -a;
            if (a >= TROPHY_ROTATE_ANGLE)
            {
                LineFollow_Stop();
                ChassisPos_ResetVelocityPid();
                ChassisPos_ResetEncoder();
                HWT101_getAngle_Reset();
                HeadingHold_Reset();
                Servo4_UseTask2Parameters();
                trophy_phase = 2;
            }
            else
            {
                wheel_target[1] =  TROPHY_ROTATE_SPEED;
                wheel_target[2] =  TROPHY_ROTATE_SPEED;
                wheel_target[3] = -TROPHY_ROTATE_SPEED;
                wheel_target[4] = -TROPHY_ROTATE_SPEED;
            }
        }
        else if ((trophy_phase == 3U) || (trophy_phase == 5U) || (trophy_phase == 7U))
        {
            /* 左移寻杯: 模式2(0x02)视觉找奖杯, 命中即停车夹取;
             * 走满距离仍未见则兜底按原固定距离进入下一阶段. */
            int hc;

            if (!trophy_cam_mode_on)
            {
                /* 铁律: 摄像头可能仍在模式循环内, 先 0x04 退出再切 0x02 */
                (void)MaxiCam_SetMode(MAXICAM_MODE_STOP);
                (void)MaxiCam_SetMode(MAXICAM_MODE_ABC);
                trophy_cam_mode_on = 1U;
                trophy_det_confirm = 0U;
            }

            if (!pos_control_active)
            {
                int dist = (trophy_phase == 3U) ? (int)TROPHY_LEFT1_DISTANCE
                         : (trophy_phase == 5U) ? (int)TROPHY_LEFT2_DISTANCE
                         :                        (int)TROPHY_LEFT3_DISTANCE;
                ChassisPos_Start(-dist, dist, -dist, dist);
            }

            if (TrophyDetect_Seen() && (trophy_det_count < TROPHY_DETECT_COUNT))
                trophy_det_count++;

            if (trophy_det_count >= TROPHY_DETECT_COUNT)
            {
                /* 识别满3次: 停车, 小爪子全部夹住 */
                LineFollow_Stop();
                ChassisPos_ResetVelocityPid();
                Servo4_LittleAllClose();
                trophy_cam_mode_on = 0U;
                trophy_ticks = 0U;
                trophy_phase++;
            }
            else
            {
                hc = HeadingHold_GetFreshCorrection();
                if (ChassisPos_Update(0, TROPHY_LEFT_SPEED, hc))
                {
                    LineFollow_Stop();
                    ChassisPos_ResetVelocityPid();
                    trophy_cam_mode_on = 0U;
                    trophy_ticks = 0U;
                    trophy_phase++;
                }
            }
        }
        else
        {
            /* 阶段 2(后退) 与 9(右后撤): 盲走 */
            int max_fwd, max_left;
            int pos_t1, pos_t2, pos_t3, pos_t4;

            if (!pos_control_active)
            {
                if (trophy_phase == 2U)
                {
                    pos_t1 = pos_t2 = pos_t3 = pos_t4 = -(int)TROPHY_BACK_DISTANCE;
                }
                else
                {
                    int back = (int)TROPHY_FINAL_BACK_DISTANCE;
                    int right = (int)TROPHY_FINAL_RIGHT_DISTANCE;
                    pos_t1 =  right - back;
                    pos_t2 = -right - back;
                    pos_t3 =  right - back;
                    pos_t4 = -right - back;
                }
                ChassisPos_Start(pos_t1, pos_t2, pos_t3, pos_t4);
            }

            if (trophy_phase == 2U)
                { max_fwd = TROPHY_BACK_SPEED; max_left = 0; }
            else
                { max_fwd = TROPHY_RIGHT_BACK_SPEED; max_left = TROPHY_RIGHT_SPEED; }
            hc = HeadingHold_GetFreshCorrection();
            if (ChassisPos_Update(max_fwd, max_left, hc))
            {
                LineFollow_Stop();
                ChassisPos_ResetVelocityPid();
                trophy_ticks   = 0;
                trophy_phase++;
            }
        }
    }

    if (route_state == ROUTE_VISION_ALIGN)
    {
        int vision_target[4];
        VisionAlignment_GetWheelTargets(vision_target);
        LineFollow_SetWheelTargets(vision_target[0], vision_target[1],
                                   vision_target[2], vision_target[3]);
    }

    if (route_state == ROUTE_TRACK_FIRST)
    {
        int i, black_count, line_state, turn_target;
        Sensor_Get();

        if (mode_entry_tick > 0)
        {
            LineFollow_Stop();
            mode_entry_tick++;
            if (mode_entry_tick >= 100)
            {
                ChassisPos_ResetEncoder();
                HWT101_getAngle_Reset();
                HeadingHold_Reset();
                ChassisPos_SetRotate90Target();
                Task1Color_Stop();
                route_state = ROUTE_ROTATE_90;
                mode_entry_tick = 0;
                lcd_div = 0;
                ChassisPos_DrawStatus(1, "Rotate 90");
            }
        }
        else if (GS_state[1] && GS_state[2] && GS_state[3] && GS_state[4])
        {
            LineFollow_Stop();
            ChassisPos_ResetVelocityPid();
            mode_entry_tick = 1;
        }
        else
        {
            black_count = GS_state[1] + GS_state[2] + GS_state[3] + GS_state[4]
                        + GS_state[5] + GS_state[6] + GS_state[7] + GS_state[8];
            if (black_count == 0)
                turn_target = last_steer;
            else {
                line_state = 0;
                for (i = 1; i <= 8; i++)
                    if (GS_state[i]) line_state += line_weights[i];
                turn_target = LineFollow_ComputeLinePID(line_state);
                last_steer = turn_target;
            }
            debug_line_state = line_state;
            debug_turn_target = turn_target;
            LineFollow_SetWheelTargets(TRACK_FIRST_SPEED_TARGET + turn_target,
                                       TRACK_FIRST_SPEED_TARGET + turn_target,
                                       TRACK_FIRST_SPEED_TARGET - turn_target,
                                       TRACK_FIRST_SPEED_TARGET - turn_target);
        }
    }

    if (route_state == ROUTE_TRACK_RETURN)
    {
        int i, black_count, line_state, turn_target;
        Sensor_Get();

        /* Rear 4 sensors (5-8) all on black + suppression expired → TROPHY */
        if (GS_state[5] && GS_state[6] && GS_state[7] && GS_state[8]
            && marker_suppress_ticks >= MARKER_SUPPRESS_TICKS)
        {
            ChassisPos_ResetVelocityPid();
            HeadingHold_Reset();
            LineFollow_Stop();
            ChassisPos_ResetEncoder();
            HWT101_getAngle_Reset();
            trophy_phase = 0;
            trophy_ticks = 0;
            trophy_cam_mode_on = 0U;
            trophy_det_confirm = 0U;
            trophy_det_last_seq = 0U;
            trophy_det_count = 0U;
            trophy_det_gap_until_ms = 0U;
            route_state = ROUTE_TROPHY;
            mode_entry_tick = 0;
            lcd_div = 0;
            ChassisPos_DrawStatus(1, "TROPHY - STOP");
            Move_Control(0, 0, 0, 0);
            goto skip_track_return;
        }

        black_count = GS_state[1] + GS_state[2] + GS_state[3] + GS_state[4]
                    + GS_state[5] + GS_state[6] + GS_state[7] + GS_state[8];
        if (black_count == 0)
            turn_target = last_steer;
        else {
            line_state = 0;
            for (i = 1; i <= 8; i++)
                if (GS_state[i]) line_state += line_weights[i];
            turn_target = LineFollow_ComputeLinePID(line_state);
            last_steer = turn_target;
        }
        debug_line_state = line_state;
        debug_turn_target = turn_target;
        /* 左右列差速转向：W1/W2同侧，W3/W4同侧 */
        LineFollow_SetWheelTargets(LINE_FOLLOW_SPEED_TARGET + turn_target,
                                   LINE_FOLLOW_SPEED_TARGET + turn_target,
                                   LINE_FOLLOW_SPEED_TARGET - turn_target,
                                   LINE_FOLLOW_SPEED_TARGET - turn_target);
        skip_track_return: ;
    }

    ChassisMotor_ControlTick(wheel_target);
}
