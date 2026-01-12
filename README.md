# Embedded / 嵌入式软件

[简体中文](#简体中文) | [English](#english)

## 简体中文

### 简介
本目录包含 EDispense 系统运行所需的**固件**与底层**嵌入式软件**，涵盖通信协议、传感器接口、电机控制与热管理等。

### 内容
- **MCU Firmware（MCU 固件）**：控制 MCU 或 FPGA，执行运动控制与系统级任务的代码。
- **Motor Control（电机控制）**：步进/伺服等运动电机控制相关软件。
- **Thermal Control（温控）**：PCB 预热与温度调节管理软件。
- **Communication（通信）**：硬件模块间的底层通信协议（如 SPI、I2C）。

### 目的
本目录负责实现嵌入式侧的实时控制与硬件协同，支撑高效、精确的点胶与热过程管理。

## English

### Introduction
This directory contains the **firmware** and low-level **embedded software** required for the operation of the EDispense system, including communication protocols, sensor interfaces, motor control, and thermal regulation.

### Contents
- **MCU Firmware**: Code for controlling the microcontroller or FPGA to execute motion control and other system-level tasks.
- **Motor Control**: Software for controlling motors, including stepper or servo motors for motion.
- **Thermal Control**: Software managing the thermal preheating and temperature regulation for the PCB.
- **Communication**: Low-level communication protocols for interfacing between hardware modules (e.g., SPI, I2C).

### Purpose
This directory is responsible for the embedded software that ensures the real-time operation and coordination of all hardware components, enabling efficient and accurate dispensing and thermal management.
