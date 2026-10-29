# 车型智能搬运机器人 · STM32F407 固件

![Language](https://img.shields.io/badge/language-C-00599C)
![MCU](https://img.shields.io/badge/MCU-STM32F407%20%40%20168%20MHz-green)
![IDE](https://img.shields.io/badge/IDE-Keil%20MDK%205-orange)
![Control](https://img.shields.io/badge/control-200%20Hz%20ISR-blueviolet)
![License](https://img.shields.io/badge/license-MIT-lightgrey)

> 2026 中国机器人大赛暨 RoboCup 中国赛 · 工程竞技类 · **车型智能搬运**赛项
> **国赛三等奖 · 专项赛三等奖**
>
> 四轮麦克纳姆轮底盘固件：三环 PID 运动控制 + 24 状态比赛路线状态机 + UART 视觉伺服对准 + 8 路舵机机构时序。

---

## 项目概览

这是一台自主搬运机器人的完整比赛固件。整车需要在无人干预下完成 **扫码取任务 → 循迹行进 → 视觉对准抓取目标物 → 机构搬运 → 定点投放** 的全流程，全程由 STM32F407 单芯片实时调度。

| 指标 | 数值 |
| --- | --- |
| 主控 | STM32F407，168 MHz（HSI + PLL，FLASH 5 WS） |
| 底盘控制周期 | **200 Hz**（TIM7 硬实时中断，5 ms） |
| 任务调度周期 | **~100 Hz**（主循环 + 视觉/扫码非阻塞轮询） |
| 比赛路线状态 | **24** 个状态节点 |
| 固件代码量 | ~7 800 行 C（BSP + Core，不含 ST HAL/CMSIS） |
| 里程标定 | **1850 counts/cm**（实测；理论 2315，差异见下文） |
| 迭代记录 | 持续迭代，提交历史含完整调参过程 |

---

## 系统架构

```mermaid
graph TD
    subgraph 硬实时层 200 Hz
        T7[TIM7 中断<br/>5 ms] --> PS[ProjectService<br/>比赛路线状态机]
        PS --> CP[ChassisPos<br/>位置环 · 梯形加减速]
        PS --> LF[LineFollow<br/>8 路灰度巡线]
        PS --> VA[VisionAlignment<br/>视觉伺服对准]
        CP --> CM[ChassisMotor<br/>速度环 ×4 轮]
        LF --> CM
        VA --> CM
        IMU[HWT101 陀螺仪<br/>UART5] --> HH[HeadingHold<br/>航向环]
        HH --> CM
        CM --> PWM[TIM8 PWM<br/>4× TB6612]
    end

    subgraph 任务层 ~100 Hz
        MAIN[main loop] --> CT[CompetitionTask]
        CT --> QR[CompetitionQr<br/>GM65 扫码解析]
        CT --> VA
        CT --> PS
    end

    subgraph 外设
        GS[前/后 8 路数字灰度] --> LF
        CAM[MaixCAM 视觉<br/>UART4 115200] --> VA
        SCAN[GM65 扫码器<br/>USART1 9600] --> QR
        T9[TIM9 软件 PWM] --> SV[8 路舵机<br/>大臂/小臂/转盘/5 小爪]
    end
```

设计上刻意把**感知、决策、执行**解耦：视觉模块只输出"四轮目标速度"，从不直接碰电机；UART 回调统一在 `HWT101.c` 分发；任何 UART 发送、LCD 访问都不会阻塞 5 ms 控制中断。这些约束写在 [`ARCHITECTURE.md`](ARCHITECTURE.md) 里，并在整个开发过程中作为评审基线。

---

## 三环 PID 控制

固件名称里的"三环"是三条互相嵌套的反馈回路，分别解决**走得准、走得直、走得稳**：

### 1. 速度环（内环）· `BSP/ChassisMotor.c`

四个轮子各自独立一个**增量式**速度 PID，在 200 Hz 中断里运行，输入是编码器差分速度，输出直接给 TIM8 的 PWM 占空比。

一个容易被忽略的细节：增量式 PID 在目标为零时不会自动归零，残余输出会让车在"暂停"状态下继续蠕动。因此代码在四轮目标全为零时**强制复位 PID 并输出 0**，保证停车是真停：

```c
if ((target_1 | target_2 | target_3 | target_4) == 0)
{
    ChassisMotor_ResetSpeedPid();
    Move_Control(0, 0, 0, 0);
    return;
}
```

（另外硬件上负 PWM 才对应前进，`Move_Control` 里统一取反，这行标定注释保留在代码中。）

### 2. 位置环（中环）· `BSP/ChassisPos.c`

负责所有**定点运动**：前进 N cm、横移 N cm、原地转 90°。实现为**梯形加减速 + 到位判定**，关键参数：

| 参数 | 值 | 含义 |
| --- | --- | --- |
| `POS_ERROR_THRESHOLD` | 600 counts | 到位误差阈值 ≈ **3 mm**（600 / 1850） |
| `POS_VEL_THRESHOLD` | 25 counts/5 ms | 到位速度阈值 ≈ **2.7 cm/s** |
| `POS_DEBOUNCE_TICKS` | 15 | 连续 15 个周期满足才判定到位（75 ms 消抖） |
| `POS_STALL_TICKS` | 150 | 堵转检测：750 ms 无位移增量即置故障 |
| `POS_ENCODER_TICK_LIMIT` | 1600 | 单周期编码器跳变异常剔除 |

到位判定同时看**误差**和**速度**，避免"冲过头又弹回来"被误判为到位；再加 75 ms 消抖。

### 3. 航向环（外环）· `BSP/HeadingHold.c`

用 HWT101 陀螺仪的 yaw 角做闭环，把航向修正量叠加到左右轮差速上，解决麦轮打滑导致的**直行跑偏**。

| 参数 | 值 |
| --- | --- |
| Kp / Ki / Kd | 3.0 / 3.1 / 0.12 |
| 输出限幅 | ±125 |
| 积分限幅 | ±44 |
| 死区 | 2.0° |

死区的作用是抑制静止时的角度抖动；输出限幅保证航向修正永远不会压过正常的转向指令。

---

## 比赛路线状态机

`BSP/ProjectService.c`（约 1 400 行）是一台 24 状态的显式状态机，每条边都对应赛场上一个可观测的动作完成条件（编码器里程到达 / 灰度特征命中 / 视觉标志位 / 陀螺仪角度到阈值 / 定时器超时）。

```
出发段    HOME_LAUNCH → HOME_FIXED_FORWARD → HOME_FIXED_PAUSE
          → HOME_FIXED_LEFT → HOME_FIXED_SETTLE
巡线段    TRACK_FIRST → TRACK_RETURN → TROPHY → ROTATE_90 → CURVE
对准段    REAR_ALIGN → PAUSE → REAR_ALIGN_2 → PAUSE_2
          VISION_ALIGN → VISION_FORWARD → VISION_BACK
终点段    FINAL_LEFT_FORWARD → FINAL_ROTATE_30 → FINAL_FORWARD_2
          → FINAL_ROTATE_ALIGN → FINAL_LEFT_40 → FINAL_ROTATE_180 → PAUSE_GENERIC
```

状态机是**单线程、无阻塞**的：每个状态只做"下发目标 + 检查完成条件"，所有耗时动作（视觉等帧、舵机到位、LCD 刷新）都靠轮询推进，绝不 `HAL_Delay` 卡住 200 Hz 中断。

---

## 视觉伺服对准

MaixCAM 通过 UART4 回传目标坐标，`VisionAlignment.c` 做**非阻塞同心圆对准**：

1. 请求视觉进入对准模式（1 字节命令）；
2. 逐帧读取坐标，与图像中心比较，生成横向/纵向的增量修正；
3. 连续 N 帧落在容差带内才判定"对准完成"，再切到固定航向直进 3 cm；
4. 帧数据带**时间戳**（`MaxiCam_GetLatest(..., max_age_ms)`），超过 50 ms 的旧帧直接丢弃——避免用过期坐标做闭环导致振荡。

### 视觉串口帧格式（10 字节，115200 8N1）

| 字节 | 字段 | 说明 |
| --- | --- | --- |
| 0–1 | `0xFF 0xAA` | 帧头 |
| 2 | `data1` | 状态/模式码（检测到目标 / 颜色码 / 奖杯码） |
| 3 | `data2` | 对准标志（1 = 已在中心容差带内） |
| 4–5 | `data3` | 目标 X 坐标，uint16 大端 |
| 6–7 | `data4` | 目标 Y 坐标，uint16 大端 |
| 8–9 | `0xFE 0xBB` | 帧尾 |

接收端是**逐字节状态机**（`MaxiCam_OnRxByte`），不是"攒够 10 字节再解析"：帧头错位会自动回退重同步，UART 溢出错误会清空缓冲重新对齐。这是长时间跑车不丢帧的关键。

---

## 硬件接口分配

| 外设 | 接口 | 参数 |
| --- | --- | --- |
| 四轮编码器 | TIM2 / TIM3 / TIM4 / TIM5 | 500 PPR × 4 倍频 × 1:28，77 mm 轮径 |
| 四轮电机 | TIM8 PWM → 4× TB6612 | 200 Hz 速度环输出 |
| 底盘控制节拍 | TIM7 | 200 Hz，抢占优先级 1 |
| 8 路舵机 | TIM9 软件 PWM（PC2/PC3/PA4/PA5/PA6/PA7/PC4/PC5） | 500–2500 µs，优先级 0 |
| 视觉 MaixCAM | UART4（PC10/PC11） | 115200 8N1 |
| 陀螺仪 HWT101 | UART5 | 115200 8N1 |
| 扫码器 GM65 | USART1 | 9600 8N1 |
| 灰度传感器 | GPIO | 前 8 路 + 后 8 路 |
| LCD | FSMC 并口 | 调试状态显示 |

机构侧是 **8 路舵机**：大臂、小臂、360° 转盘（CH2，锁死/解锁接口用于防止搬运中物块掉落）、5 个小爪子（统控开合）。舵机参数按"任务 1 / 任务 2"两套姿态切换，视觉启动不允许改变舵机参数——这条约束是为了避免机构动作和视觉时序互相干扰。

---

## 目录结构

```
.
├── BSP/                       # 业务层（核心）
│   ├── ProjectService.c       # 比赛路线状态机（~1400 行）
│   ├── ChassisPos.c           # 位置环：梯形加减速 + 到位判定
│   ├── ChassisMotor.c         # 速度环：4 路增量 PID
│   ├── HeadingHold.c          # 航向环：IMU yaw 闭环
│   ├── LineFollow.c/.h        # 巡线 + 全路线距离/速度常量表
│   ├── VisionAlignment.c      # 视觉伺服对准
│   ├── MaxiCam.c              # 视觉串口帧解析（字节流状态机）
│   ├── CompetitionQr.c        # 任务编号解析
│   ├── GM65Scanner.c          # 扫码器驱动
│   ├── Servo4.c               # 8 路舵机软件 PWM + 机构时序
│   ├── HWT101.c               # 陀螺仪 + UART 回调统一分发
│   ├── Sensor.c               # 前/后 8 路灰度
│   ├── LCD.c / LCD_FONT.c     # FSMC 屏显
│   └── TestMode.h             # 调试开关集中管理
├── Core/                      # CubeMX 生成 + 中断
├── Drivers/                   # ST HAL / CMSIS
├── MDK-ARM/                   # Keil 工程
├── ARCHITECTURE.md            # 模块职责与强制约束
└── BanyunCommonF407.ioc       # CubeMX 工程
```

---

## 构建与烧录

**环境**：Keil MDK 5（需 STM32F4xx DFP 包）+ STM32CubeMX（仅修改外设配置时）。

```
1. 用 Keil 打开  MDK-ARM/BanyunCommonF407.uvprojx
2. 选择目标 F407，编译（F7）
3. ST-Link / J-Link 下载（F8）
```

外设配置改动请用 CubeMX 打开 `BanyunCommonF407.ioc` 重新生成，**不要手改生成的初始化代码**。

编译产物（`*.axf / *.hex / *.o / *.map`、Keil 个人 GUI 状态）已全部写入 `.gitignore`，仓库里只有源码与工程文件。

`TestMode.h` 里集中了调试开关（舵机行程测试、色块舵机联调、视觉跳过等），比赛版本全部关闭。这些测试分支留在代码里是有意的——现场排查机构问题时可以直接改宏重烧。

---

## 关于 1850 counts/cm

代码里所有距离常量都基于 **1850 counts/cm** 的实测标定，而理论值是 2315 counts/cm（77 mm 轮径、500 PPR、4 倍频、1:28 减速比）。差异来自麦轮打滑、轮子受压形变、地面摩擦与编码器安装误差。

`LineFollow.h` 顶部专门写了警告：**改距离常量前必须实车重标定，不要拿理论值直接换算替换**，否则所有定点运动都会偏。这个坑在调试期真实踩过，所以留下了注释。

---

## 可改进方向

- 状态机的转移条件目前是"硬编码常量 + 手工调参"，可以引入参数表与在线调参（UART/蓝牙）以缩短现场调试时间。
- 位置环目前纯编码器推算，长时间运行会累积里程误差；可融合 IMU 做里程计。
- 视觉对准是"逐帧增量修正"，可改为预测式控制（Kalman / 落点前馈）以提升高速下的对准稳定性。

---

## 团队与分工

**固件、底盘三环控制、比赛路线状态机、机构舵机时序**由 **康维嘉** 负责（本仓库全部提交均由其本人提交）。

比赛程序是在队伍既有工程的基础上迭代而来：本人在学长学姐离队、无人接手的情况下接过一队车辆，**仅有一周时间**完成参数标定与全流程调试，最终获国赛三等奖。本仓库的提交历史即为这一过程。

视觉端基础工程由学长 **mtc** 编写、后由本人接手迭代，见配套仓库 [`robocup-smart-carry-vision`](https://github.com/vega-thrive/robocup-smart-carry-vision)；机械结构与硬件电路由队伍其他成员完成。

同一赛项的另一辆车（二队）是**从零独立实现**的另一套固件，53 状态 / 12 336 行，见 [`robocup-smart-carry-firmware-team2`](https://github.com/vega-thrive/robocup-smart-carry-firmware-team2)。两套代码不存在复用关系。

## License

本项目代码基于 [MIT License](LICENSE) 开源。
`Drivers/` 下的 ST HAL / CMSIS 组件版权归 STMicroelectronics 所有，遵循其原有许可。
