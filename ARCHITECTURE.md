# 搬运小车软件结构

当前比赛程序只保留六条实际运行链路：

1. `CompetitionTask`：约 100 Hz 主循环调度，只调用扫码、视觉和底盘。
2. `LineFollow`：完整比赛路线状态机，发布四轮目标速度。
3. `ChassisMotor`：200 Hz 编码器采样、增量速度 PID 和电机输出。
4. `GM65Scanner` + `CompetitionQr`：USART1 扫码与任务编号/顺序解析。
5. `MaxiCam` + `VisionAlignment`：UART4 帧协议与同心圆非阻塞对准。
6. `Servo4` + `MechanismSequence`：TIM9 软件 PWM 和两组已标定舵机参数。

## 硬件接口

- USART1：GM65，9600 8N1。
- UART4：MaxiCam，115200 8N1，PC10/PC11。
- UART5：HWT101，115200 8N1。
- TIM7：底盘 200 Hz 控制中断，优先级 1。
- TIM9：四路舵机软件 PWM，PC2、PC3、PA4、PA5，优先级 0。
- TIM2/3/4/5：四轮编码器。
- TIM8：四轮电机 PWM。

## 强制约束

- UART 回调只在 `HWT101.c` 中统一分发。
- UART发送、LCD访问和状态切换准备不得阻塞 TIM7。
- 视觉模块只生成四轮目标，不直接调用电机驱动。
- 舵机参数由底盘阶段选择；视觉启动不得切换舵机参数。
- 二维码原始接收属于 `GM65Scanner`，比赛编号范围和默认策略属于
  `CompetitionQr`。
- 不重新启用 CAN、旧软件 I²C、PF8/PF9 舵机测试或步进电机代码。

## 不参与编译的历史文件

以下文件保留在磁盘仅供参考，不在 Keil 目标中：

- `Core/Src/can.c`、`Core/Inc/can.h`
- `BSP/CAN_MOTO.*`、`BSP/Moto_CAN.*`
- `BSP/IIC.*`
- `BSP/StepControl.*`、`BSP/Timer.*`
- `BSP/ServoPwm.*`、`BSP/ServoChannelSearch.*`
- `LineFollow.c.bak`、`LineFollow.h.bak`
