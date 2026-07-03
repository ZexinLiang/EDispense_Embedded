# EDispense Embedded / STM32F407 底层固件

[简体中文](#简体中文) | [English](#english)

---

## 简体中文

### 项目简介

EDispense Embedded 是 EDispense 自动点锡/点胶系统的底层电机控制板固件，运行在 STM32F407VET6 上。固件负责 CoreXY 平面运动、Z 轴高度控制、挤锡执行、串口屏交互、USB CDC 调试以及与 RK3588 上位机的实时通信。

本仓库包含 STM32CubeMX 工程、Keil MDK-ARM 工程、CMake/ARM GCC 构建配置、FreeRTOS 任务代码和用户运动控制模块。

### 硬件与工具链

- **MCU**: STM32F407VET6, ARM Cortex-M4 @ 168 MHz
- **RTOS**: FreeRTOS / CMSIS-OS
- **运动结构**: CoreXY + Z 轴 + 挤锡轴
- **开发工具**: STM32CubeMX, Keil MDK-ARM, CMake + ARM GCC
- **调试接口**: USB CDC 虚拟串口、UART、Vofa+
- **上位机接口**: RK3588 通过 UART 协议下发点锡任务

### 主要功能

- CoreXY 双电机联动运动学正/逆解算
- X/Y/Z/挤锡多轴步进电机控制
- 梯形加减速与阻塞/非阻塞运动接口
- 激光测距闭环 Z 轴高度定位
- 挤锡轴定量挤出与回抽
- 陶晶驰/TJC 串口屏触摸与轨迹显示
- RK3588 上位机命令解析与 ACK 回传
- USB CDC 调试命令与状态上报
- Vofa+ FireWater 实时数据观测
- FreeRTOS 多任务调度

### 目录结构

```text
EDispense/
├── Core/                         # STM32CubeMX 生成的 HAL/应用入口
│   ├── Inc/
│   └── Src/                      # main.c, freertos.c, 外设初始化等
├── Drivers/                      # STM32 HAL 与 CMSIS
├── Middlewares/                  # FreeRTOS / USB Device Library
├── USB_DEVICE/                   # USB CDC 设备配置
├── User/                         # 用户运动控制与屏幕模块
│   ├── Stepper.c/.h              # 步进电机控制、加减速、状态机
│   └── TJC.c/.h                  # 串口屏协议与 UI 处理
├── MDK-ARM/                      # Keil 工程
├── cmake/                        # ARM GCC CMake 工具链配置
├── CMakeLists.txt
├── CMakePresets.json
├── EDispense.ioc                 # STM32CubeMX 工程
└── STM32F407XX_FLASH.ld          # 链接脚本
```

### FreeRTOS 任务

| 任务 | 作用 |
|---|---|
| `Task_UI` | 屏幕刷新、轨迹显示、USB/Vofa 状态上报 |
| `Task_CoreXY` | 主状态机、XY 运动、自动点锡流程调度 |
| `TASK_Z_Axis` | Z 轴运动与激光高度闭环 |
| `Task_Squeeze` | 挤锡轴动作执行 |

### 通信协议概览

#### RK3588 UART 协议

固件通过 UART 接收 RK3588 上位机下发的自动作业、急停、复位、Z 轴标定等命令，并通过 ACK/状态帧反馈执行结果。

#### USB CDC 调试协议

USB CDC 用于桌面调试、单轴移动、急停/回零、Z 轴步进与挤锡测试。

### 构建

#### CMake / ARM GCC

```bash
cmake --preset Debug
cmake --build --preset Debug
```

或：

```bash
cmake -B build -G Ninja
cmake --build build
```

#### Keil MDK-ARM

打开 `MDK-ARM/EDispense.uvprojx`，然后在 Keil 中编译、下载与调试。

### 开源协议

本项目采用 MIT License，详见 [LICENSE](LICENSE)。

---

## English

### Overview

EDispense Embedded is the low-level firmware for the motor-control board of the EDispense automatic solder-paste dispensing system. It runs on an STM32F407VET6 MCU and handles CoreXY motion, Z-axis height control, solder extrusion, touchscreen interaction, USB CDC debugging, and real-time communication with the RK3588 host computer.

This repository includes the STM32CubeMX project, Keil MDK-ARM project, CMake/ARM GCC build configuration, FreeRTOS task code, and user motion-control modules.

### Hardware and Toolchain

- **MCU**: STM32F407VET6, ARM Cortex-M4 @ 168 MHz
- **RTOS**: FreeRTOS / CMSIS-OS
- **Motion platform**: CoreXY + Z axis + solder extrusion axis
- **Tools**: STM32CubeMX, Keil MDK-ARM, CMake + ARM GCC
- **Debug interfaces**: USB CDC virtual COM port, UART, Vofa+
- **Host interface**: RK3588 sends dispensing jobs through UART

### Features

- CoreXY forward/inverse kinematics
- Multi-axis stepper control for X/Y/Z/extrusion axes
- Trapezoidal acceleration and blocking/non-blocking motion APIs
- Laser-distance-based closed-loop Z-height positioning
- Quantified solder extrusion and retract motion
- TJC serial touchscreen input and path visualization
- RK3588 command parser and ACK feedback
- USB CDC debug command and status telemetry
- Vofa+ FireWater real-time telemetry output
- FreeRTOS-based multi-task scheduling

### Repository Layout

```text
EDispense/
├── Core/                         # STM32CubeMX-generated HAL/application code
│   ├── Inc/
│   └── Src/                      # main.c, freertos.c, peripheral init, etc.
├── Drivers/                      # STM32 HAL and CMSIS
├── Middlewares/                  # FreeRTOS / USB Device Library
├── USB_DEVICE/                   # USB CDC device configuration
├── User/                         # User motion-control and display modules
│   ├── Stepper.c/.h              # Stepper control, acceleration and state machine
│   └── TJC.c/.h                  # Serial touchscreen protocol and UI handling
├── MDK-ARM/                      # Keil project
├── cmake/                        # ARM GCC CMake toolchain configuration
├── CMakeLists.txt
├── CMakePresets.json
├── EDispense.ioc                 # STM32CubeMX project
└── STM32F407XX_FLASH.ld          # Linker script
```

### FreeRTOS Tasks

| Task | Purpose |
|---|---|
| `Task_UI` | Display refresh, path drawing, USB/Vofa telemetry |
| `Task_CoreXY` | Main state machine, XY motion, automatic dispensing workflow |
| `TASK_Z_Axis` | Z-axis motion and laser height control |
| `Task_Squeeze` | Solder extrusion execution |

### Protocol Overview

#### RK3588 UART Protocol

The firmware receives automatic-job, emergency-stop, reset, and Z-calibration commands from the RK3588 host through UART, and returns ACK/status frames after execution.

#### USB CDC Debug Protocol

USB CDC is used for desktop debugging, single-axis movement, emergency stop/home, Z stepping, and extrusion tests.

### Build

#### CMake / ARM GCC

```bash
cmake --preset Debug
cmake --build --preset Debug
```

Or:

```bash
cmake -B build -G Ninja
cmake --build build
```

#### Keil MDK-ARM

Open `MDK-ARM/EDispense.uvprojx`, then build, flash, and debug in Keil.

### License

This project is licensed under the MIT License. See [LICENSE](LICENSE).
