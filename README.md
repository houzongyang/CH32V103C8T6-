# CH32V103C8T6 无人车底盘控制

本仓库保存 CH32V103C8T6 无人车底盘控制工程、调参版本、A1 M8 激光雷达资料，以及 FPGA+MCU 底盘控制器参考资料。

主控为 WCH CH32V103C8T6 RISC-V MCU。当前代码集成舵机转向、后轮电机 PWM、RPLIDAR A1M8 360 度激光雷达、JY901 姿态传感器、WT530D 激光测距、OLED 显示，并实现基于雷达点云的锥桶/门框识别与通过控制状态机。

## 目录说明

| 路径 | 说明 |
| --- | --- |
| `CH32V103C8T6/` | 主工程。包含 MounRiver Studio 工程文件、源码、启动文件、外设库、链接脚本和已生成的编译产物。 |
| `CH32V103C8T6_右侧比左侧好的3/` | 调参版本。代码结构与主工程基本一致，主要用于保留“右侧比左侧好的”实车调参结果。 |
| `A1 M8激光雷达/` | RPLIDAR A1M8 资料、SDK、测试工具和 STM32 示例程序压缩包。 |
| `无人车底盘控制器相关参考文档与文件/` | FPGA+MCU 底盘控制器参考文档、硬件规格书、原理图、示例程序和芯片手册。 |
| `*.zip` | 工程或资料的原始压缩包/备份包。 |

## 工程版本差异

两个 CH32V103C8T6 工程大部分源码相同，关键调参差异集中在 `User/main.c` 和 `User/cone_detect.c`。

| 工程 | `pass_time_ms` | `target_gate_count` | 用途倾向 |
| --- | ---: | ---: | --- |
| `CH32V103C8T6/` | `1600` | `4` | 通过 4 个门后进入完成状态的主版本。 |
| `CH32V103C8T6_右侧比左侧好的3/` | `3000` | `0` | 右侧效果较好的调参版本，不按门数量自动停止。 |

如果需要继续实车调参，优先改 `User/main.c` 中的 `drive_cfg` 参数，例如：

- `speed_turn`
- `speed_approach`
- `speed_centering`
- `speed_pass`
- `pass_time_ms`
- `center_start_distance_mm`
- `max_steer_angle`
- `yaw_sign`
- `target_gate_count`

## 核心代码结构

| 文件 | 作用 |
| --- | --- |
| `User/main.c` | 系统入口。初始化时钟、中断、舵机、电机、OLED、WT530D、JY901、RPLIDAR 和锥桶通过状态机。主循环执行雷达任务、导航状态机、姿态任务和测距显示。 |
| `User/rplidar_360.c/.h` | RPLIDAR A1M8 驱动。通过 USART1 + DMA 接收雷达数据，整理为 360 个角度箱数据。 |
| `User/cone_detect.c/.h` | 锥桶/门框识别和车辆通过状态机。状态包括 `SEARCH`、`APPROACH`、`PASS`、`CLEAR_GATE`、`DONE`。 |
| `User/Motor.c/.h` | 后轮电机 PWM 输出。 |
| `User/Servo.c/.h` | 舵机 PWM 输出和 `turnAngle()` 转角控制。 |
| `User/JY901.c/.h` | JY901 姿态传感器 USART2 + DMA 接收、帧解析、航向角滤波和相对航向归零。 |
| `User/Laser.c/.h` | WT530D 激光测距 USART3 + DMA 接收、距离解析和 OLED 显示。 |
| `User/Oled.c/.h` | OLED 显示驱动。 |
| `User/SR04.c/.h` | 超声波测距相关代码，当前主流程中主要预留。 |
| `User/GPS.c/.h` | 早期 GPS 解析代码。当前 `main.c` 中显示的是固定坐标模拟值。 |

## 主要硬件接口

| 模块 | 接口 | 引脚/通道 | 备注 |
| --- | --- | --- | --- |
| RPLIDAR A1M8 | USART1 + DMA | PA9 TX, PA10 RX, DMA1 Channel5 | 波特率 `115200`。 |
| JY901 | USART2 + DMA | PA2 TX, PA3 RX, DMA1 Channel6 | 波特率 `9600`。启动后有效航向数据稳定且运行约 1.5 s 后归零。 |
| WT530D 激光测距 | USART3 + DMA | PB10 TX, PB11 RX, DMA1 Channel3 | 波特率 `115200`。PB10 也可用于 `printf` 输出。 |
| 舵机转向 | TIM3 PWM | PA6, TIM3_CH1 | 20 ms 周期，`turnAngle(-45..45)` 映射到约 1000 到 2000 us。 |
| 后轮电机 | TIM1 PWM | PA8, TIM1_CH1 | 约 10 kHz PWM，`Car_SetRearSpeed(0..100)` 设置占空比。 |
| OLED | 软件 I2C | PB6 SCL, PB7 SDA | 从机地址写值为 `0x78`。 |
| SR04 超声波 | GPIO/TIM2 | Trig PA4, Echo PA15 或 PA3 | 代码里存在两个初始化版本，使用前需按实际接线确认。 |

## 编译和烧录

推荐使用 MounRiver Studio 打开和编译工程。

1. 安装 MounRiver Studio 2.x、WCH RISC-V 工具链和 WCH-Link 驱动。
2. 在 MounRiver Studio 中打开或导入：
   - `CH32V103C8T6/CH32V103C8T6.wvproj`
   - 或 `CH32V103C8T6_右侧比左侧好的3/CH32V103C8T6.wvproj`
3. 选择目标工程后执行 Build。
4. 使用 WCH-Link 下载或调试。

已生成的固件通常位于工程内的 `obj/` 或 `dbg/` 目录，例如：

- `CH32V103C8T6/obj/CH32V103C8T6.hex`
- `CH32V103C8T6/obj/CH32V103C8T6.elf`
- `CH32V103C8T6_右侧比左侧好的3/obj/CH32V103C8T6.hex`
- `CH32V103C8T6_右侧比左侧好的3/obj/CH32V103C8T6.elf`

### 命令行编译注意

工程内 `obj/makefile` 是 MounRiver Studio 自动生成文件，其中链接脚本 `Link.ld` 可能带有旧电脑上的绝对路径。如果直接运行 `make` 报找不到 `Link.ld`，建议回到 MounRiver Studio 重新生成工程配置，或将 makefile 中的 `-T ".../Ld/Link.ld"` 改为当前工程下的 `../Ld/Link.ld`。

示例：

```powershell
cd CH32V103C8T6\obj
make clean
make all
```

## 运行逻辑

上电后主程序会依次执行：

1. 配置中断优先级、系统时钟和延时函数。
2. 初始化舵机并回中，初始化后轮电机并置零速。
3. 初始化 OLED，并显示模拟 GPS 坐标。
4. 初始化 WT530D、JY901 和 RPLIDAR。
5. 初始化锥桶检测和 `ConeDrive` 状态机。
6. 启动 1 ms SysTick 计时。
7. 在主循环中持续执行雷达数据处理、锥桶/门框识别、车辆控制、JY901 姿态更新和激光测距显示。

OLED 当前会在模拟 GPS 页面和传感器/状态页面之间切换。代码中 `show_gps_screen` 初始为 1，GPS 页面显示约 2 s，随后切换到传感器页面约 10 s。

## 调试建议

- 先确认舵机方向：`turnAngle()` 中负角为左转，正角为右转。如果实车方向相反，先检查舵机安装和连线，再调控制符号。
- 如果车头右偏却继续右打方向，可尝试把 `drive_cfg.yaw_sign` 从 `1` 改为 `-1`。
- `target_gate_count = 0` 表示不按通过门数量自动进入 `DONE`。
- `target_gate_count > 0` 时，通过数量达到目标后进入 `DONE` 并停车。
- RPLIDAR 可用性不足时，状态机会继续搜索或使用当前帧数据，实车前应确认雷达供电、串口线序和 115200 波特率。
- 源码中部分中文注释可能在不同终端或编辑器里显示乱码。修改前建议确认文件编码，避免一次性转换导致无关 diff。

## 参考资料

相关硬件资料集中放在：

- `A1 M8激光雷达/`
- `无人车底盘控制器相关参考文档与文件/FPGA+MCU无人车控制器参考文档/`
- `无人车底盘控制器相关参考文档与文件/FPGA+MCU控制器程序实例/`

这些资料包含 RPLIDAR 文档、SDK、CH32V103/CH32V208 手册、FPGA 控制板说明、原理图、IO 对应表和示例程序。
