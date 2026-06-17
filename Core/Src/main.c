/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "Stepper.h"
#include "TJC.h"        // 串口屏
#include <stdio.h>      // sscanf
#include <string.h>
#include <stdlib.h>     // atoi 函数解析激光距离
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define one_div_sqrt_2 0.7071067811865f   // 1/根号2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
Stepper stepper1 = {0}; // X轴电机
Stepper stepper2 = {0}; // Y轴电机
Stepper stepper3 = {0}; // Z轴电机
Stepper stepper4 = {0}; // 挤锡电机
Stepper stepper5 = {0}; // 预留
uint8_t RxBuffer[50] = {0}; // 串口屏不定长数据缓冲区
uint8_t RxBuffer2[50] = {0}; // RK3588不定长数据缓冲区
extern DMA_HandleTypeDef hdma_usart1_rx; // DMA 句柄
extern DMA_HandleTypeDef hdma_usart2_rx; // DMA 句柄
extern DMA_HandleTypeDef hdma_usart6_rx; // DMA 句柄
uint8_t laser_rx_buf[50] = {0}; // 激光测距接收缓冲区
uint32_t laser_dis = 0; // 激光测距当前距离
uint8_t vofa_rx_byte = 0; // UART2可靠接收缓冲区


// --- 状态机 & 作业变量 ---
volatile SystemState_t g_SystemState = STATE_IDLE;
volatile AutoCycleSubState_t g_AutoCycleSubState = CYCLE_START;
SolderJob_t g_CurrentJob;
volatile uint8_t g_NewJobReceived = 0;

// 激光Z轴标定
float g_LaserCalibrationOffset = 0.0f; // 由标定命令设置


// --- 坐标变量 ---
float device_x = 0;         // 软件记录的绝对X坐标
float device_y = 0;         // 软件记录的绝对Y坐标
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void delta_dis_transform(float delta_x, float delta_y, float* motor_delta_x, float* motor_delta_y);
void Get_Real_Position(float *real_x, float *real_y);
void move_axis_to(float target_x, float target_y);
void Send_RK3588_Ack(uint8_t cmd_id, uint8_t payload); // 发送ACK到RK3588
void Laser_Dis_Update(void);           // 激光测距更新
void Send_Telemetry_To_Vofa(float x, float y, uint32_t z, uint8_t state);
void Send_Log_To_Vofa(const char* msg); 
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief CoreXY 正向运动学变换
 */
void delta_dis_transform(float delta_x, float delta_y, float* motor_delta_x, float* motor_delta_y){
	*motor_delta_x = (delta_x - delta_y) * one_div_sqrt_2;
	*motor_delta_y = (delta_x + delta_y) * one_div_sqrt_2;
}

/**
 * @brief CoreXY 逆向运动学: 获取实际坐标位置
 */
void Get_Real_Position(float *real_x, float *real_y) {
    float m1_phys = -(stepper1.position_ctnow * stepper1.stepangle);
    float m2_phys = -(stepper2.position_ctnow * stepper2.stepangle);

    float m1_rel = m1_phys - 0.0f;
    float m2_rel = m2_phys - 500.0f;

    *real_x = (m1_rel + m2_rel) * one_div_sqrt_2;
    *real_y = (m2_rel - m1_rel) * one_div_sqrt_2;
}

/**
 * @brief 移动到绝对坐标
 */
void move_axis_to(float target_x, float target_y){
    if(target_x > 2475.0f) target_x = 2475.0f; else if(target_x < 0) target_x = 0;
    if(target_y > 2475.0f) target_y = 2475.0f; else if(target_y < 0) target_y = 0;

    float current_x, current_y;
    Get_Real_Position(&current_x, &current_y); 

    float delta_x = target_x - current_x;
    float delta_y = target_y - current_y;

    device_x = target_x;
    device_y = target_y;

    float motor_move_x, motor_move_y;
    delta_dis_transform(delta_x, delta_y, &motor_move_x, &motor_move_y);

    StpDistanceSetBlocking(&stepper1, motor_move_x, 450, 200);
    StpDistanceSetBlocking(&stepper2, motor_move_y, 450, 200);
}

/**
 * @brief  使用激光传感器将Z轴(stepper3)精确下降到目标高度
 * @param  target_height_mm PCB上方的目标最终高度(mm)
 */
void Move_Z_Axis_To_Height(float target_height_mm) {
    uint32_t timeout_start = HAL_GetTick();
    const float TOLERANCE = 0.5f; // 目标误差0.5mm以内停止

    while (HAL_GetTick() - timeout_start < 15000) { // 15秒超时
        float current_dis = (float)laser_dis;
        
        if (current_dis < 10.0f) { // 传感器安全检查
            Stepper_Stop(&stepper3);
            return; 
        }

        // 计算误差(需要移动的距离)
        float error = current_dis - target_height_mm;

        // 1. 到位检查
        if (error <= TOLERANCE && error >= -TOLERANCE) {
            Stepper_Stop(&stepper3); 
            return; 
        }

        float move_step = 0.0f;
        uint32_t speed = 800; // 较快的基础速度

        // 2. 逻辑: 若当前距离160,目标120,
        // 误差+40,需要向下移动(负方向)
        
        if (error > 10.0f) {
            move_step = 8.0f;  // 远: 向下移动8mm
            speed = 1200;       // 高速
        } 
        else if (error > 2.0f) {
            move_step = 2.0f;  // 接近: 向下移动2mm
            speed = 600; 
        }
        else if (error > 0.0f) {
            move_step = 0.5f;  // 很接近: 向下移动0.5mm
            speed = 300; 
        }
        else if (error < -10.0f) {
            move_step = -8.0f;   // 过低: 向上移动8mm
            speed = 1200;
        }
        else if (error < -2.0f) {
            move_step = -2.0f;   // 过低: 向上移动2mm
            speed = 600;
        }
        else if (error < 0.0f) {
            move_step = -0.5f;   // 很接近: 向上移动0.5mm
            speed = 300;
        }

        // 执行移动
        // 阻塞模式: 等待8mm或0.5mm移动完成后才返回
        StpDistanceSetBlocking(&stepper3, move_step, speed, 400); 
        
        // 给激光传感器一点时间稳定,消除移动振动影响
        osDelay(10); 
    }
    
    Stepper_Stop(&stepper3);
}

/**
 * @brief 将Z轴缩回到安全高度
 */
void Retract_Z_Axis(void) {
    // 假设正值向上移动,上移10mm
    StpDistanceSetBlocking(&stepper3, 10.0f, 400, 200); 
}

/**
 * @brief 驱动挤锡电机(stepper4)进行指定次数的挤压
 * @param count 挤压次数
 */
void Squeeze_Solder(uint16_t count) {
    // 单次挤压的微动距离
    const float SQUEEZE_DISPENSE_AMOUNT_MM = 40.0f; // 根据实际调整
    const float SQUEEZE_RETRACT_AMOUNT_MM = -20.0f; // 轻微回抽防止漏锡

    for (uint16_t i = 0; i < count; i++) {
        // 假设正值挤出
        StpDistanceSetBlocking(&stepper4, SQUEEZE_DISPENSE_AMOUNT_MM, 400, 50);
        osDelay(20); // 短暂暂停
        StpDistanceSetBlocking(&stepper4, SQUEEZE_RETRACT_AMOUNT_MM, 400, 50);
        osDelay(50); // 两次挤压之间等待
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM10_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
	// 启动步进控制相关定时器
	HAL_TIM_Base_Start_IT(&htim8);
	HAL_TIM_Base_Start_IT(&htim10);
	// STP1: DIR->PA5 STP->PA4 USE_TIMER TIM8_CHANNEL1
	Init_Stepper(&stepper1,DIR1_GPIO_Port,DIR1_Pin,STP1_GPIO_Port,STP1_Pin,&htim8,TIM_CHANNEL_1,0.1125f);
	// STP2: DIR->PA7 STP->PA6 USE_TIMER TIM8_CHANNEL2
	Init_Stepper(&stepper2,DIR2_GPIO_Port,DIR2_Pin,STP2_GPIO_Port,STP2_Pin,&htim8,TIM_CHANNEL_2,0.1125f);
	// STP3: DIR->PD9 STP->PD8 USE_TIMER TIM8_CHANNEL3
	Init_Stepper(&stepper3,DIR3_GPIO_Port,DIR3_Pin,STP3_GPIO_Port,STP3_Pin,&htim8,TIM_CHANNEL_3,0.1125f);
	// STP4: DIR->PD10 STP->PD10 USE_TIMER TIM8_CHANNEL4
	Init_Stepper(&stepper4,DIR4_GPIO_Port,DIR4_Pin,STP4_GPIO_Port,STP4_Pin,&htim8,TIM_CHANNEL_4,0.1125f);
	// STP5: DIR->PE1 STP->PE0 USE_TIMER TIM10_CHANNEL1
	Init_Stepper(&stepper5,DIR5_GPIO_Port,DIR5_Pin,STP5_GPIO_Port,STP5_Pin,&htim10,TIM_CHANNEL_1,0.1125f);
	
	// 启动不定长数据接收
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1,RxBuffer,50);
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT); //关闭过半中断
	
	  // 2. Start RK3588/Vofa+ Reception (UART2) 
	HAL_UART_Receive_IT(&huart2, &vofa_rx_byte, 1);

	// 3. Start Laser Sensor Reception (UART6) 
	HAL_UARTEx_ReceiveToIdle_DMA(&huart6, laser_rx_buf, 50);
	__HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT); 

    TJC_Init(&huart1); // 初始化串口屏
    
    #warning "暂时注释掉初始电机复位，测试USB"
    // HAL_Delay(10000); // 等待系统稳定
    //
    // // Y轴初始移动到起始点
    // StpDistanceSetBlocking(&stepper2, 500.0f, 450.0f, 200.0f); 
    //
    // // 等待运动完成
    // while(IFMOVING(stepper2.motor_state)) {
    //     HAL_Delay(10); 
    // }

    // 同步坐标
    device_x = 0.0f;
    device_y = 0.0f;
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_ui_tick = 0;

  while (1)
  {
      
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
volatile uint8_t g_TJC_Status_Update = 0; 
volatile uint8_t g_AckPending = 0;

// --- VOFA/RK3588 状态机 ---
static uint8_t vofa_step = 0;
static uint8_t vofa_cmd = 0;
static uint8_t vofa_len = 0;
static uint8_t vofa_payload[20]; // 最大payload长度为20字节
static uint8_t vofa_cnt = 0;
static uint8_t vofa_sum = 0;

void Vofa_RxCallback(uint8_t rx_data) {
    switch (vofa_step) {
        case 0: if (rx_data == 0xBB) vofa_step = 1; break;
        case 1: 
            if (rx_data == 0x66) vofa_step = 2; 
            else if (rx_data == 0xBB) vofa_step = 1; // 如果是连续BB则保持
            else vofa_step = 0; 
            break;
        case 2:
            vofa_cmd = rx_data; vofa_sum = rx_data; vofa_step = 3; break;
        case 3:
            vofa_len = rx_data; vofa_sum += rx_data; vofa_cnt = 0;
            if (vofa_len > 20) { vofa_step = 0; break; } // 数组安全限制
            if (vofa_len > 0) vofa_step = 4;
            else vofa_step = 5; 
            break;
        case 4:
            vofa_payload[vofa_cnt++] = rx_data; vofa_sum += rx_data;
            if (vofa_cnt >= vofa_len) vofa_step = 5;
            break;
        case 5:
            if (vofa_sum == rx_data) { // 校验和通过!
                if (vofa_cmd == 0xEE) {
                    g_SystemState = STATE_EMERGENCY_STOP; 
                    Stepper_Stop(&stepper1); Stepper_Stop(&stepper2); 
                    Stepper_Stop(&stepper3); Stepper_Stop(&stepper5);
                    
                    g_TJC_Status_Update = 1; // 通知RTOS更新文本
                    g_AckPending = 0xEE;     // 通知RTOS发送ACK
                }
                else if (vofa_cmd == 0xFF) {
                    g_SystemState = STATE_IDLE;
                    g_NewJobReceived = 0; 
                    TJC_TargetX = 0.0f; TJC_TargetY = 0.0f; TJC_MoveFlag = 1;  
                    
                    g_TJC_Status_Update = 2; // 通知RTOS清除文本
                    g_AckPending = 0xFF;     // 通知RTOS发送ACK
                }   
                else if (vofa_cmd == 0x10 && vofa_len == 14) {
                    memcpy(&g_CurrentJob.target_x, &vofa_payload[0], 4);
                    memcpy(&g_CurrentJob.target_y, &vofa_payload[4], 4);
                    memcpy(&g_CurrentJob.target_z, &vofa_payload[8], 4);
                    memcpy(&g_CurrentJob.squeeze_count, &vofa_payload[12], 2);
                    g_NewJobReceived = 1; 
                } 
                else if (vofa_cmd == 0x20 && vofa_len == 4) {
                    memcpy(&g_LaserCalibrationOffset, &vofa_payload[0], 4);
                    g_AckPending = 0x80;
                }
            }
            vofa_step = 0; 
            break;
    }
}

// -------------------------------------------------------------

// ============================================================
// USB CDC 调试协议 (复刻 TJC AA55 帧, 独立状态机, 与串口屏 TJC 并存)
// 下行帧: AA 55 | CMD | PAYLOAD | CHECKSUM(=cmd+payload各字节累加和 & 0xFF)
//   0x01 XY绝对移动 (int16 x, int16 y, 单位0.1mm)
//   0x02 急停
//   0x03 回零/解除急停
//   0x06 Z轴步进     (int16 steps, 正=上/负=下)
//   0x07 挤锡        (uint8 count)
// 上行帧:
//   0x10 状态(周期100ms): int16 x, int16 y, int16 z(激光), uint8 state(0空闲/1运动/2急停)
//   0x11 动作完成: uint8 cmd_id
// 注意: Z步进/挤锡为阻塞动作, 不在USB回调(中断上下文)执行, 仅置标志由 StartTask02 主任务执行
// ============================================================
// USB -> 主任务 的标志位 (供 freertos.c StartTask02 消费)
volatile uint8_t  g_UsbZMoveFlag   = 0;
volatile int16_t  g_UsbZSteps      = 0;
volatile uint8_t  g_UsbZHomeFlag   = 0;  // Z轴回零标志
volatile uint8_t  g_UsbSqueezeFlag = 0;
volatile uint8_t  g_UsbSqueezeCount= 0;

// 上行: 发送状态帧 0x10
void Send_USB_Status(int16_t x, int16_t y, int16_t z, int16_t laser, uint8_t state){
    uint8_t b[13];
    b[0]=0xAA; b[1]=0x55; b[2]=0x10;
    b[3]=(x>>8)&0xFF;     b[4]=x&0xFF;
    b[5]=(y>>8)&0xFF;     b[6]=y&0xFF;
    b[7]=(z>>8)&0xFF;     b[8]=z&0xFF;
    b[9]=(laser>>8)&0xFF; b[10]=laser&0xFF;
    b[11]=state;
    uint8_t sum=0; for(int i=2;i<12;i++) sum+=b[i];
    b[12]=sum;
    CDC_Transmit_FS(b, 13);
}

// 上行: 发送动作完成帧 0x11
void Send_USB_Done(uint8_t cmd_id){
    uint8_t b[5];
    b[0]=0xAA; b[1]=0x55; b[2]=0x11; b[3]=cmd_id;
    b[4]=(0x11+cmd_id)&0xFF;
    CDC_Transmit_FS(b, 5);
}

// USB 命令处理: CDC天然整包接收, 直接索引解析 (帧: AA 55 | CMD | PAYLOAD | SUM)
void USB_Data_Process_Callback(uint8_t* Buf, uint32_t *Len){
    uint32_t n = Len[0];
    if(n < 4) return;                       // 最短帧 AA 55 CMD SUM = 4字节
    if(Buf[0]!=0xAA || Buf[1]!=0x55) return; // 帧头校验
    uint8_t id = Buf[2];
    uint8_t need;                            // payload长度
    if(id==0x01)      need=4;
    else if(id==0x06) need=2;
    else if(id==0x07) need=1;
    else              need=0;                // 0x02/0x03
    if(n < (uint32_t)(3+need+1)) return;     // 帧长不足
    uint8_t* p = &Buf[3];                    // payload起始
    // 累加和校验: cmd + payload
    uint8_t sum = id;
    for(uint8_t i=0;i<need;i++) sum += p[i];
    if(sum != Buf[3+need]) return;           // 校验失败丢弃

    switch(id){
        case 0x01:  // XY绝对移动 (复用TJC全局变量)
            if(g_SystemState!=STATE_EMERGENCY_STOP){
                int16_t ix=(int16_t)((p[0]<<8)|p[1]);
                int16_t iy=(int16_t)((p[2]<<8)|p[3]);
                TJC_TargetX=(float)ix/10.0f;
                TJC_TargetY=(float)iy/10.0f;
                TJC_MoveFlag=1;
            }
            break;
        case 0x02:  // 急停
            g_SystemState=STATE_EMERGENCY_STOP;
            Stepper_Stop(&stepper1); Stepper_Stop(&stepper2);
            Stepper_Stop(&stepper3); Stepper_Stop(&stepper5);
            break;
        case 0x03:  // 回零/解除急停: XY回(0,0) + Z回0
            if(g_SystemState==STATE_EMERGENCY_STOP) g_SystemState=STATE_IDLE;
            TJC_TargetX=0; TJC_TargetY=0; TJC_MoveFlag=1;
            g_UsbZHomeFlag=1;
            break;
        case 0x06:  // Z轴步进 (置标志, 主任务执行)
            if(g_SystemState!=STATE_EMERGENCY_STOP){
                g_UsbZSteps=(int16_t)((p[0]<<8)|p[1]);
                g_UsbZMoveFlag=1;
            }
            break;
        case 0x07:  // 挤锡 (置标志, 主任务执行)
            if(g_SystemState!=STATE_EMERGENCY_STOP){
                g_UsbSqueezeCount=p[0];
                g_UsbSqueezeFlag=1;
            }
            break;
    }
}

// 步进电机OC中断回调
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim8  && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) StepperInOC(&stepper1);
	if (htim == &htim8  && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) StepperInOC(&stepper2);
	if (htim == &htim8  && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) StepperInOC(&stepper3);
	if (htim == &htim8  && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) StepperInOC(&stepper4);
	if (htim == &htim10 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) StepperInOC(&stepper5);
}

// 封装一个向 RK3588 发送状态反馈的函数
void Send_RK3588_Ack(uint8_t cmd_id, uint8_t payload) {
    uint8_t tx_buf[6];
    tx_buf[0] = 0xBB; // 帧头
    tx_buf[1] = 0x66;
    tx_buf[2] = cmd_id; // 反馈ID
    tx_buf[3] = 0x01;   // 长度 1 字节
    tx_buf[4] = payload;
    tx_buf[5] = tx_buf[2] + tx_buf[3] + tx_buf[4]; // 校验和
    HAL_UART_Transmit(&huart2, tx_buf, 6, 20);
}

// 1. UART2 专用回调 (不受步进电机超限错误影响!)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        Vofa_RxCallback(vofa_rx_byte); // 发送字节到状态机
        
        // 重新使能中断以立即捕获下一个字节
        HAL_UART_Receive_IT(&huart2, &vofa_rx_byte, 1);
    }
}

// 2. IDLE DMA CALLBACK FOR UART1 and UART6
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
    
    // --- 1. TJC Screen on UART1 ---
    if(huart->Instance == USART1){
        uint8_t temp_buf[50];
        memcpy(temp_buf, RxBuffer, Size);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, RxBuffer, 50); 
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
        
        for(uint16_t i = 0; i < Size; i++) TJC_RxCallback(temp_buf[i]);
    }
    
    // --- 3. Laser Sensor on UART6 ---
    if(huart->Instance == USART6) {
        laser_rx_buf[Size < 50 ? Size : 49] = '\0'; 
        Laser_Dis_Update(); 
        
        HAL_UARTEx_ReceiveToIdle_DMA(&huart6, laser_rx_buf, 50);
	    __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
    }
}

// --- 滤波后的激光测距更新 ---
void Laser_Dis_Update(void) {
    char *ptr = strstr((char *)laser_rx_buf, "nd: ");
    if (ptr == NULL) ptr = strstr((char *)laser_rx_buf, "d: ");
    
    if (ptr != NULL) {
        int offset = (strncmp(ptr, "nd: ", 4) == 0) ? 4 : 3;
        int32_t dist = atoi(ptr + offset);
        
        if (dist > 0) {
            // 简易软件低通滤波 (指数移动平均)
            if (laser_dis == 0) laser_dis = (uint32_t)dist;
            else laser_dis = (laser_dis * 3 + (uint32_t)dist) / 4;
        }
    }
}

// 发送激光数据到 Vofa+ (UART2)
void Send_Telemetry_To_Vofa(float x, float y, uint32_t z, uint8_t state) {
    char tx_buf[64];
    // 格式完全匹配 FireWater: "data:ch0,ch1,ch2,ch3\n"
    int len = snprintf(tx_buf, sizeof(tx_buf), "data:%.1f,%.1f,%lu,%d\n", x, y, z, state);
    HAL_UART_Transmit(&huart2, (uint8_t*)tx_buf, len, 20);
}

// --- 文本日志记录 ---
void Send_Log_To_Vofa(const char* msg) {
    char tx_buf[64];
    int len = snprintf(tx_buf, sizeof(tx_buf), "log:%s\n", msg);
    HAL_UART_Transmit(&huart2, (uint8_t*)tx_buf, len, 20);
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
