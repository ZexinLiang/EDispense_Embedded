#include "TJC.h"
#include "stepper.h" 
#include <stdio.h>
#include <string.h>

extern Stepper stepper1;
extern Stepper stepper2;
extern Stepper stepper3;
extern Stepper stepper5;

extern volatile SystemState_t g_SystemState; // 使用全局状态
volatile UI_Page_t current_ui_page = PAGE_MAIN; // 独立跟踪UI屏幕页面

uint8_t TJC_MoveFlag = 0;
float TJC_TargetX = 0;
float TJC_TargetY = 0;

static UART_HandleTypeDef *tjc_huart;

static uint8_t rx_step = 0;
static uint8_t rx_id = 0;
static uint8_t rx_payload[4];
static uint8_t rx_cnt = 0;
static uint8_t calc_sum = 0;

static uint16_t last_dbg_x = 0, last_dbg_y = 0;
static uint16_t last_main_x = 0, last_main_y = 0;
static uint8_t force_update_path = 1;

void TJC_Init(UART_HandleTypeDef *huart) {
    tjc_huart = huart;
}

void TJC_RxCallback(uint8_t rx_data) {
    switch (rx_step) {
        case 0: if (rx_data == 0xAA) rx_step = 1; break;
        case 1: if (rx_data == 0x55) rx_step = 2; else rx_step = 0; break;
        case 2:
            rx_id = rx_data; calc_sum = rx_id; rx_cnt = 0;
            if (rx_id == 0x01) rx_step = 3; else rx_step = 4;
            break;
        case 3:
            rx_payload[rx_cnt++] = rx_data; calc_sum += rx_data;
            if (rx_cnt >= 4) rx_step = 4;
            break;
        case 4:
            if (calc_sum == rx_data) {
                char buf[64];
                switch(rx_id) {
                    case 0x01: 
                        if (g_SystemState != STATE_EMERGENCY_STOP) { // 统一检查
                            int16_t ix = (rx_payload[0] << 8) | rx_payload[1];
                            int16_t iy = (rx_payload[2] << 8) | rx_payload[3];
                            TJC_TargetX = (float)ix / 10.0f;
                            TJC_TargetY = (float)iy / 10.0f;
                            TJC_MoveFlag = 1;
                        }
                        break;
                    case 0x02: 
                        g_SystemState = STATE_EMERGENCY_STOP; 
                        Stepper_Stop(&stepper1); 
                        Stepper_Stop(&stepper2);
                        Stepper_Stop(&stepper3);
                        Stepper_Stop(&stepper5);
                        g_TJC_Status_Update = 1; // 延迟UI更新
                        break;
                    case 0x03: 
                        if (g_SystemState == STATE_EMERGENCY_STOP) { 
                            g_SystemState = STATE_IDLE;
                            g_TJC_Status_Update = 2; // 延迟UI更新
                        }
                        TJC_TargetX = 0; TJC_TargetY = 0; 
                        TJC_MoveFlag = 1;
                        break;
                    case 0x04: 
                        current_ui_page = PAGE_DEBUG; // UI页面与系统状态分离
                        sprintf(buf, "fill 349,101,100,75,0\xff\xff\xff");
                        HAL_UART_Transmit(tjc_huart, (uint8_t*)buf, strlen(buf), 20);
                        force_update_path = 1;
                        break;
                    case 0x05: 
                        current_ui_page = PAGE_MAIN; // UI页面与系统状态分离
                        sprintf(buf, "fill 285,103,150,71,0\xff\xff\xff");
                        HAL_UART_Transmit(tjc_huart, (uint8_t*)buf, strlen(buf), 20);
                        force_update_path = 1;
                        break;
                }
            }
            rx_step = 0;
            break;
    }
}

// 核心UI更新引擎
void TJC_ProcessUI(float real_x, float real_y, uint8_t is_moving) {
    if (g_SystemState == STATE_EMERGENCY_STOP) return;

    char buf[128];

    // 1. 更新 Debug 界面上的坐标数值
    if (current_ui_page == PAGE_DEBUG) {
        sprintf(buf, "debug.tx_curr.txt=\"%.1f\"\xff\xff\xff"
                     "debug.ty_curr.txt=\"%.1f\"\xff\xff\xff", 
                     real_x, real_y);
        HAL_UART_Transmit(tjc_huart, (uint8_t*)buf, strlen(buf), 20);
    }

    // 2. 只有在移动时才画线
    if (is_moving || force_update_path) {
        
        if (current_ui_page == PAGE_DEBUG) {
            uint16_t cur_px = 449 - (uint16_t)(real_y * 100.0f / 2475.0f);
            uint16_t cur_py = 176 - (uint16_t)(real_x * 75.0f  / 2475.0f);
            
            if (cur_px < 349) cur_px = 349; else if (cur_px > 449) cur_px = 449;
            if (cur_py < 101) cur_py = 101; else if (cur_py > 176) cur_py = 176;

            if (!force_update_path && (cur_px != last_dbg_x || cur_py != last_dbg_y)) {
                sprintf(buf, "line %d,%d,%d,%d,65504\xff\xff\xff", last_dbg_x, last_dbg_y, cur_px, cur_py);
                HAL_UART_Transmit(tjc_huart, (uint8_t*)buf, strlen(buf), 20);
            }
            last_dbg_x = cur_px; 
            last_dbg_y = cur_py;
        } 
        else if (current_ui_page == PAGE_MAIN) {
            uint16_t cur_px = 435 - (uint16_t)(real_y * 150.0f / 2475.0f);
            uint16_t cur_py = 174 - (uint16_t)(real_x * 71.0f  / 2475.0f);
            
            if (cur_px < 285) cur_px = 285; else if (cur_px > 435) cur_px = 435;
            if (cur_py < 103) cur_py = 103; else if (cur_py > 174) cur_py = 174;

            if (!force_update_path && (cur_px != last_main_x || cur_py != last_main_y)) {
                sprintf(buf, "line %d,%d,%d,%d,65504\xff\xff\xff", last_main_x, last_main_y, cur_px, cur_py);
                HAL_UART_Transmit(tjc_huart, (uint8_t*)buf, strlen(buf), 20);
            }
            last_main_x = cur_px; 
            last_main_y = cur_py;
        }
        force_update_path = 0; 
    }
}
