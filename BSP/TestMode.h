#ifndef __TEST_MODE_H__
#define __TEST_MODE_H__

/* ================================================================
 * 测试模式总开关 —— 比赛前只检查本文件即可
 * 0 = 正式比赛程序 (完整任务1+任务2)
 * ================================================================ */

#define SERVO_RANGE_TEST_MODE   0   /* CH2 舵机量程扫描测试 */
#define COLOR_SERVO_TEST_MODE   0   /* 转盘颜色识别测试 */
#define SERVO_CHANNEL_TEST_MODE 0   /* CH2 固定脉宽测试 */
#define TASK1_ROUTE_TEST_MODE   0   /* 仅跑任务1路段, 任务2扫码点停车 */

/* ---- 测试分支专用常量 (仅对应测试模式生效) ---- */
#define COLOR_SERVO_CHANNEL     2U
#define COLOR_SERVO_HOME_US   600U
#define COLOR_SERVO_STEP_US   400U  /* 72 degrees on the measured 360-degree servo */
#define COLOR_CAMERA_BOOT_MS  4000U

#define SERVO_TEST_LOW_US       500U
#define SERVO_TEST_HIGH_US      1600U  /* 仅量程扫描模式使用 */

#endif
