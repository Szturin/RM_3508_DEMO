/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "pid.h"
#include "remote_control.h"
#include "string.h"
#include "bsp_system.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern uint8_t UART_Buffer[100];
extern Motor_measure_t Motor_measure[14];
extern Motor_PID_typedef Motor_pid_SPEED[14];
int16_t delta;					 //设定速度与实际速度的差值
int16_t max_speed_change = 50; //电机单次最大变化速度，加减速用
								 // 500经测试差不多是最大加速区间，即从零打到最大速度不异常的最大值
float set_speed = 500;	 //电机速度全局变量
float set_speed_temp;			 //加减速时的临时设定速度
#define CAN_CONTROL // CAN总线控制
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT_IDLE(&huart1, UART_Buffer, 100); //启动串口3接收
	can_filter_init();
	PID_Total_Init();
	scheduler_init();
	HAL_Delay(1000);
	printf("{plotter}%d\n", Motor_measure[0].speed);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	while (1)
	{
		scheduler_run();
		//key_proc();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		// demo for M2006
		// spd = remote_control.ch1 * 6000/ 660;
#if defined CAN_CONTROL	//CAN总线控制
		// PID采样，获取电机数据
		// PS:在CAN接收中断中更新数据
		// 变化量缓冲处理
		static uint8_t i = 0;
		
		//10ms一次缓冲
		if( ++i >= 2)
		{
			//电机变速缓冲器
			LimitAcceleration(set_speed, Motor_measure[0].speed, 350, &set_speed_temp);
			i=0;
		}
		// 计算电机的PID输出值
		//单环（增量式）控制
		Motor_measure[0].Output = Motor_PID_calc(&Motor_pid_SPEED[0], Motor_measure[0].speed, set_speed_temp);
	
		// 将PID输出作用于电机
		Set_motor_cmd(&hcan1, First_STDID, Motor_measure[0].Output, 0, 0, 0);
		// 通过串口调试助手观察实际速度与设定速度.
//		printf("{PID out:%f}\n", (float)Motor_measure[0].Output);
//		printf("{real speed:%d}\n", Motor_measure[0].speed);
//		printf("{set speed:%f}\n", set_speed_temp);
		
		printf("Out:%f,%d,%f\n", (float)Motor_measure[0].Output,Motor_measure[0].speed,set_speed_temp);

#elif defined PWM_CONTROL // PWM控制
		if (key_cnt < 0)
			key_cnt = 0;
		if (key_cnt > 20)
			key_cnt = 20;
		set_speed[0] = set_speed[1] = set_speed[2] = set_speed[3] = key_cnt * 100; // 0-2000,单向转
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, set_speed[0]);
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, set_speed[1]);
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, set_speed[2]);
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, set_speed[3]);
#endif
		HAL_Delay(5); // 处理周期
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
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

#ifdef  USE_FULL_ASSERT
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
