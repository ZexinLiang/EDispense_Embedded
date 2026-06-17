# EDispense Embedded

基于 STM32F407 的桌面级焊锡膏自动点胶机嵌入式固件。

## 硬件平台

- **主控**: STM32F407VET6 (Cortex-M4, 168MHz)
- **RTOS**: FreeRTOS (CMSIS-OS v2)
- **工具链**: ARM GCC (CMake) / Keil MDK-ARM

## 功能概述

- **CoreXY 运动控制**: 支持 S 形梯形加减速的步进电机驱动
- **多轴联动**: X/Y/Z/挤锡四轴，支持阻塞/非阻塞两种运动模式
- **激光测距**: 通过 UART 读取激光传感器数据，实现 Z 轴精确高度控制
- **串口屏交互**: 陶晶驰 (TJC) 串口屏，支持触摸坐标输入与轨迹显示
- **上位机通信**: 
  - RK3588 上位机 (UART2, BB66 协议帧)
  - USB CDC 虚拟串口调试 (AA55 协议帧)
  - Vofa+ FireWater 实时数据波形显示
- **FreeRTOS 多任务架构**:
  - `Task_UI`: 串口屏刷新 / 遥测发送 / USB状态上报 (50ms周期)
  - `Task_CoreXY`: 主状态机 (空闲 / 调试移动 / 自动循环 / 急停)
  - `Task_Z_Axis`: Z轴专控
  - `Task_Squeeze`: 挤锡专控

## 目录结构

```
EDispense/
├── Core/                   # STM32CubeMX 生成的外设驱动
│   ├── Inc/                # 头文件 (main.h, FreeRTOSConfig.h ...)
│   └── Src/                # 源文件 (main.c, freertos.c ...)
├── Drivers/                # HAL 库 & CMSIS
├── Middlewares/             # FreeRTOS / USB Device Library
├── USB_DEVICE/             # USB CDC 配置
├── User/                   # 用户自定义模块
│   ├── stepper.c/.h        # 步进电机驱动 (梯形加减速 + 速度跟随)
│   └── TJC.c/.h            # 陶晶驰串口屏驱动
├── MDK-ARM/                # Keil MDK 工程
├── cmake/                  # CMake 工具链配置
├── CMakeLists.txt          # CMake 顶层构建
└── EDispense.ioc           # STM32CubeMX 工程文件
```

## 构建

### CMake (推荐)

```bash
cmake -B build -G Ninja --preset Debug
cmake --build build
```

### Keil MDK

打开 `MDK-ARM/EDispense.uvprojx` 直接编译。

## 通信协议

### RK3588 上行协议 (BB66 帧)

| CMD  | 说明 |
|------|------|
| 0xEE | 急停 |
| 0xFF | 复位/解除急停 |
| 0x10 | 自动作业 (14字节: x/y/z float + squeeze_count uint16) |
| 0x20 | Z轴标定 (4字节: float offset) |

### USB CDC 下行协议 (AA55 帧)

| CMD  | 说明 |
|------|------|
| 0x01 | XY绝对移动 (int16 x, int16 y, 单位0.1mm) |
| 0x02 | 急停 |
| 0x03 | 回零/解除急停 |
| 0x06 | Z轴步进 (int16 steps) |
| 0x07 | 挤锡 (uint8 count) |

## License

MIT
