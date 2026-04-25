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

/* USER CODE BEGIN PV */
Stepper stepper1 = {0};
Stepper stepper2 = {0};
Stepper stepper3 = {0};
Stepper stepper4 = {0};
Stepper stepper5 = {0};
uint8_t screen_rx_buf[50] = {0}; // 串口屏不定长数据缓冲区
extern DMA_HandleTypeDef hdma_usart1_rx; // DMA 句柄
extern DMA_HandleTypeDef hdma_usart6_rx; // DMA 句柄
uint8_t laser_rx_buf[50] = {0};
uint32_t laser_dis = 0; // 激光测距测试距离
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Laser_Dis_Update(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USB_DEVICE_Init();
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
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1,screen_rx_buf,50);
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT); //关闭过半中断
	// 激光测距接收
	HAL_UARTEx_ReceiveToIdle_DMA(&huart6,laser_rx_buf,50);
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT); //关闭过半中断
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		HAL_Delay(10);
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

// USB 接收数据处理回调函数，已经在底层调用，直接在此处理数据即可
void USB_Data_Process_Callback(uint8_t* Buf, uint32_t *Len){
	CDC_Transmit_FS(Buf,Len[0]); // USB 数据回显
}

// 步进 OC 中断回调
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim8  && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) StepperInOC(&stepper1);
	if (htim == &htim8  && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) StepperInOC(&stepper2);
	if (htim == &htim8  && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) StepperInOC(&stepper3);
	if (htim == &htim8  && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) StepperInOC(&stepper4);
	if (htim == &htim10 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) StepperInOC(&stepper5);
}

// 不定长数据接收回调
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	if(huart == &huart1){
		// 串口屏数据包解码放在此处
		HAL_UART_Transmit(huart,screen_rx_buf,Size,HAL_MAX_DELAY);// 数据回显
		
		HAL_UARTEx_ReceiveToIdle_DMA(huart, screen_rx_buf, 50); //再次打开接受
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT); //关闭过半中断
	} else if(huart == &huart6){
		Laser_Dis_Update();
		HAL_UARTEx_ReceiveToIdle_DMA(huart, laser_rx_buf, 50); //再次打开接受
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT); //关闭过半中断
	}
}

void Laser_Dis_Update(void){
	if(laser_rx_buf[6] == 0x30 && laser_rx_buf[29] == 0x20){ // 接收成功 + 关键校验帧验证
		uint8_t byte1 = laser_rx_buf[25] == 0x20 ? 0 : laser_rx_buf[25] - 0x30;
		uint8_t byte2 = laser_rx_buf[26] == 0x20 ? 0 : laser_rx_buf[26] - 0x30;
		uint8_t byte3 = laser_rx_buf[27] == 0x20 ? 0 : laser_rx_buf[27] - 0x30;
		uint8_t byte4 = laser_rx_buf[28] == 0x20 ? 0 : laser_rx_buf[28] - 0x30;
		laser_dis = byte1*1000 + byte2*100 + byte3*10 + byte4; // 古法解码，单位mm
	}
	else return;
}

/* USER CODE END 4 */

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
