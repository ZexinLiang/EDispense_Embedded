/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Stepper.h"
#include "TJC.h"
#include <stdio.h>      // sprintf需要
#include <string.h>     // strlen需要
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern Stepper stepper1;
extern Stepper stepper2;

extern UART_HandleTypeDef huart1; // <--- ADD THIS LINE
extern UART_HandleTypeDef huart2; // <--- ADD THIS LINE (You will need this for Send_RK3588_Ack)

extern volatile SystemState_t g_SystemState;
extern volatile AutoCycleSubState_t g_AutoCycleSubState;
extern SolderJob_t g_CurrentJob;
extern volatile uint8_t g_NewJobReceived;
extern volatile uint32_t laser_dis;
/* USER CODE END Variables */
osThreadId Task_UIHandle;
osThreadId Task_CoreXYHandle;
osThreadId TASK_Z_AxisHandle;
osThreadId Task_SqueezeHandle;
osMessageQId CoreXY_QueueHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
extern void Get_Real_Position(float *real_x, float *real_y);
extern void move_axis_to(float target_x, float target_y);

extern void Move_Z_Axis_To_Height(float target_height_mm);
extern void Squeeze_Solder(uint16_t count);
extern void Retract_Z_Axis(void);
extern void Send_RK3588_Ack(uint8_t cmd_id, uint8_t payload);
extern void Send_Telemetry_To_Vofa(float x, float y, uint32_t z, uint8_t state);
extern void Send_Log_To_Vofa(const char* msg); 
// USB CDC 调试通信 (定义在 main.c)
extern void Send_USB_Status(int16_t x, int16_t y, int16_t z, int16_t laser, uint8_t state);
extern void Send_USB_Done(uint8_t cmd_id);
extern Stepper stepper3;
extern volatile uint32_t laser_dis;
extern volatile uint8_t  g_UsbZMoveFlag;
extern volatile int16_t  g_UsbZSteps;
extern volatile uint8_t  g_UsbZHomeFlag;
extern volatile uint8_t  g_UsbSqueezeFlag;
extern volatile uint8_t  g_UsbSqueezeCount;
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);
void StartTask04(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of CoreXY_Queue */
  osMessageQDef(CoreXY_Queue, 4, uint16_t);
  CoreXY_QueueHandle = osMessageCreate(osMessageQ(CoreXY_Queue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Task_UI */
  osThreadDef(Task_UI, StartDefaultTask, osPriorityNormal, 0, 512);
  Task_UIHandle = osThreadCreate(osThread(Task_UI), NULL);

  /* definition and creation of Task_CoreXY */
  osThreadDef(Task_CoreXY, StartTask02, osPriorityIdle, 0, 256);
  Task_CoreXYHandle = osThreadCreate(osThread(Task_CoreXY), NULL);

  /* definition and creation of TASK_Z_Axis */
  osThreadDef(TASK_Z_Axis, StartTask03, osPriorityIdle, 0, 128);
  TASK_Z_AxisHandle = osThreadCreate(osThread(TASK_Z_Axis), NULL);

  /* definition and creation of Task_Squeeze */
  osThreadDef(Task_Squeeze, StartTask04, osPriorityIdle, 0, 128);
  Task_SqueezeHandle = osThreadCreate(osThread(Task_Squeeze), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the Task_UI thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  uint32_t tick_count = 0; // 100ms间隔计数器

  /* Infinite loop */
  for(;;)
  {
      float real_x, real_y;
      Get_Real_Position(&real_x, &real_y); 
      
      uint8_t is_moving = IFMOVING(stepper1.motor_state) || IFMOVING(stepper2.motor_state);
      
       // 1. 绘制路径轨迹 (安全的)
      TJC_ProcessUI(real_x, real_y, is_moving);
	  
      // 2. 中断请求时安全更新屏幕文本
      if (g_TJC_Status_Update == 1) {
          g_TJC_Status_Update = 0;
          char buf[64];
          sprintf(buf, "main.t_status.txt=\"EMERGENCY STOPPED !\"\xff\xff\xff");
          HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 20);
          sprintf(buf, "debug.t_status.txt=\"EMERGENCY STOPPED !\"\xff\xff\xff");
          HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 20);
      } else if (g_TJC_Status_Update == 2) {
          g_TJC_Status_Update = 0;
          char buf[64];
          sprintf(buf, "main.t_status.txt=\"\"\xff\xff\xff");
          HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 20);
          sprintf(buf, "debug.t_status.txt=\"\"\xff\xff\xff");
          HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 20);
      }

      // 3. 中断请求时安全发送RK3588应答
      if (g_AckPending != 0) {
          Send_RK3588_Ack(g_AckPending, 0x01);
          g_AckPending = 0;
      }

      // 4. 发送激光测距数据
      if (++tick_count >= 2) {
          Send_Telemetry_To_Vofa(real_x, real_y, laser_dis, g_SystemState);
          tick_count = 0;
      }
      // ---------------------------------------------------------------

      // 5. USB CDC 状态帧 0x10 (每100ms): 坐标(0.1mm) + 激光 + 状态映射
      //    state: 0=空闲, 1=运动中, 2=急停
      {
          uint8_t usb_state;
          if (g_SystemState == STATE_EMERGENCY_STOP) usb_state = 2;
          else if (is_moving)                        usb_state = 1;
          else                                       usb_state = 0;
          Send_USB_Status((int16_t)(real_x * 10.0f),
                          (int16_t)(real_y * 10.0f),
                          (int16_t)(-(stepper3.stepangle * stepper3.position_ctnow) * 10.0f),
                          (int16_t)laser_dis,
                          usb_state);
      }

      // FreeRTOS 延时
      osDelay(50);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the Task_CoreXY thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
    switch (g_SystemState)
    {
        case STATE_IDLE:
            // 检查是否有来自TJC串口屏的命令(调试移动)
            if (TJC_MoveFlag) {
                TJC_MoveFlag = 0;
                g_SystemState = STATE_DEBUG_MOVE;
            }
            // USB调试: Z轴步进 (阻塞动作, 主任务执行)
            else if (g_UsbZMoveFlag) {
                g_UsbZMoveFlag = 0;
                // Z轴软限位 [0, 950]°: 向下为正方向, 超界则裁剪实际步进量
                float cur_z = -(stepper3.stepangle * stepper3.position_ctnow);  // Z坐标,向下为正
                float target_z = cur_z + (float)g_UsbZSteps;
                if (target_z < 0.0f)        target_z = 0.0f;
                else if (target_z > 950.0f) target_z = 950.0f;
                float actual = target_z - cur_z;
                if (actual != 0.0f) StpDistanceSetBlocking(&stepper3, actual, 400, 200);
                Send_USB_Done(0x06);
            }
            // USB调试: Z轴回零 (回到0°)
            else if (g_UsbZHomeFlag) {
                g_UsbZHomeFlag = 0;
                float back = stepper3.stepangle * stepper3.position_ctnow;  // 回到ctnow=0
                if (back != 0.0f) StpDistanceSetBlocking(&stepper3, back, 400, 200);
                Send_USB_Done(0x03);
            }
            // USB调试: 挤锡 (阻塞动作, 主任务执行)
            else if (g_UsbSqueezeFlag) {
                g_UsbSqueezeFlag = 0;
                Squeeze_Solder(g_UsbSqueezeCount);
                Send_USB_Done(0x07);
            }
            // 检查是否有来自RK3588的新作业
            else if (g_NewJobReceived) {
                g_NewJobReceived = 0;
                g_AutoCycleSubState = CYCLE_START; // 重置子状态
                g_SystemState = STATE_AUTO_CYCLE;
				Send_Log_To_Vofa("Starting Cycle"); // 调试日志
                // 发送ACK给RK3588
            }
            break;

        case STATE_DEBUG_MOVE:
            move_axis_to(TJC_TargetX, TJC_TargetY);
            Send_USB_Done(0x01); // USB调试: XY移动到位完成通知
            g_SystemState = STATE_IDLE; // 运动完成后返回空闲状态
            break;

        case STATE_AUTO_CYCLE:
            switch(g_AutoCycleSubState)
            {
                case CYCLE_START:
                    g_AutoCycleSubState = CYCLE_MOVING_XY;
                    break;
                case CYCLE_MOVING_XY:
                    move_axis_to(g_CurrentJob.target_x, g_CurrentJob.target_y);
                    // 安全防护: 急停在运动中触发, 立即中止!
                    if (g_SystemState == STATE_EMERGENCY_STOP) break; 
                    
                    g_AutoCycleSubState = CYCLE_LOWERING_Z;
				    Send_RK3588_Ack(0x81, 0x02);
                    break;
                case CYCLE_LOWERING_Z:
                    Move_Z_Axis_To_Height(g_CurrentJob.target_z);
                    if (g_SystemState == STATE_EMERGENCY_STOP) break; 
                    
                    g_AutoCycleSubState = CYCLE_SQUEEZING;
                    break;
                case CYCLE_SQUEEZING:
                    Squeeze_Solder(g_CurrentJob.squeeze_count);
                    if (g_SystemState == STATE_EMERGENCY_STOP) break; 
                    
                    g_AutoCycleSubState = CYCLE_RETRACTING_Z;
                    break;
                case CYCLE_RETRACTING_Z:
                    Retract_Z_Axis();
                    if (g_SystemState == STATE_EMERGENCY_STOP) break; 
                    
                    g_AutoCycleSubState = CYCLE_COMPLETE;
                    break;
                case CYCLE_COMPLETE:
                    Send_RK3588_Ack(0x81, 0x02); // 原有二进制应答
                    
                    // 发送人类可读日志到 Vofa+
                    char log_msg[50];
                    sprintf(log_msg, "Job OK X:%.1f Y:%.1f Z:%.1f", 
                            g_CurrentJob.target_x, g_CurrentJob.target_y, g_CurrentJob.target_z);
                    Send_Log_To_Vofa(log_msg);
                    
                    g_SystemState = STATE_IDLE;
                    break;
            }
            break;
        
        case STATE_CALIBRATING_Z:
            // 此状态由 vofa+ 命令进入
            // 在 vofa+ 命令解析器中设置 g_LaserCalibrationOffset
            // 目前仅等待新命令改变状态
            break;

        case STATE_EMERGENCY_STOP:
            // 此状态下机器不做任何事
            // 只能通过 RK3588/TJC 的"复位"命令退出
            break;
		
    }
    osDelay(20); // Main control loop delay
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the TASK_Z_Axis thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the Task_Squeeze thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void const * argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask04 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
