#ifndef __TJC_H
#define __TJC_H

#include "main.h"

// --- 暴露给主循环的全局变量 ---
extern volatile UI_Page_t current_ui_page;
extern uint8_t TJC_MoveFlag;     // 移动触发标志
extern float TJC_TargetX;        // 目标X坐标
extern float TJC_TargetY;        // 目标Y坐标

// --- API 接口 ---
void TJC_Init(UART_HandleTypeDef *huart);
void TJC_RxCallback(uint8_t rx_data);
void TJC_ProcessUI(float real_x, float real_y, uint8_t is_moving);

#endif /* __TJC_H */