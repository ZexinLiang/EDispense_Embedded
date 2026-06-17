/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
// 主系统状态
typedef enum {
    STATE_IDLE,                 // 等待TJC(调试)或RK3588(自动)命令
    STATE_AUTO_CYCLE,           // 执行来自RK3588的完整焊接作业
    STATE_DEBUG_MOVE,           // 执行来自TJC屏幕的单次移动
    STATE_CALIBRATING_Z,        // Z轴标定专用状态(vofa+)
    STATE_EMERGENCY_STOP        // 所有电机已停止,需复位命令才能退出
} SystemState_t;

// 自动循环子状态
typedef enum {
    CYCLE_START,
    CYCLE_MOVING_XY,
    CYCLE_LOWERING_Z,
    CYCLE_SQUEEZING,
    CYCLE_RETRACTING_Z,
    CYCLE_COMPLETE
} AutoCycleSubState_t;

// 存储来自RK3588的完整作业命令的结构体
typedef struct {
    float target_x;
    float target_y;
    float target_z;
    uint16_t squeeze_count;
} SolderJob_t;

typedef enum {
    PAGE_MAIN = 0,
    PAGE_DEBUG
} UI_Page_t;

extern volatile uint8_t g_TJC_Status_Update; 
extern volatile uint8_t g_AckPending;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void USB_Data_Process_Callback(uint8_t* Buf, uint32_t *Len);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define STP1_Pin GPIO_PIN_4
#define STP1_GPIO_Port GPIOA
#define DIR1_Pin GPIO_PIN_5
#define DIR1_GPIO_Port GPIOA
#define STP2_Pin GPIO_PIN_6
#define STP2_GPIO_Port GPIOA
#define DIR2_Pin GPIO_PIN_7
#define DIR2_GPIO_Port GPIOA
#define STP3_Pin GPIO_PIN_8
#define STP3_GPIO_Port GPIOD
#define DIR3_Pin GPIO_PIN_9
#define DIR3_GPIO_Port GPIOD
#define STP4_Pin GPIO_PIN_10
#define STP4_GPIO_Port GPIOD
#define DIR4_Pin GPIO_PIN_11
#define DIR4_GPIO_Port GPIOD
#define STP5_Pin GPIO_PIN_0
#define STP5_GPIO_Port GPIOE
#define DIR5_Pin GPIO_PIN_1
#define DIR5_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
