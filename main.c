/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define PWMfreq 10000
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t pwm=0;
uint16_t step =0;
uint8_t tx_buff[]={"robotics adt"};
uint8_t RecvBuffer[8];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void PWM_out(uint32_t freq, int16_t duty, TIM_HandleTypeDef *htim, uint32_t Channel);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*void PWM_LED()
{
		for(pwm =0;pwm<400;pwm=pwm+4)
		{
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,pwm);
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,pwm);
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,pwm);
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,pwm);
			__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,pwm);
			HAL_Delay(50);
		}
		HAL_Delay(2000);
		for(pwm =400;pwm>0;pwm=pwm-4)
		{
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,pwm);
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,pwm);
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,pwm);
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,pwm);
			__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,pwm);
			HAL_Delay(50);
		}
		HAL_Delay(2000);
}*/



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
  MX_TIM5_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart3,RecvBuffer,8);
	HAL_UART_Transmit_IT(&huart3,tx_buff,12);
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_4);
  /* USER CODE END 2 */
		HAL_GPIO_WritePin(ON0_GPIO_Port,ON0_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(ON1_GPIO_Port,ON1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(ON2_GPIO_Port,ON2_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(ON3_GPIO_Port,ON3_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(ON4_GPIO_Port,ON4_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(ON5_GPIO_Port,ON5_Pin,GPIO_PIN_SET);
	PWM_out(PWMfreq,1000,&htim5,4);	
	HAL_Delay(2000);
	PWM_out(PWMfreq,0,&htim5,4);	
	HAL_Delay(2000);
	
	

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
		if(huart->Instance==huart3.Instance)
		{
			HAL_UART_Receive_IT(&huart3,RecvBuffer,8);
		}
		
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
		__NOP();
}
void PWM_out(uint32_t freq, int16_t duty, TIM_HandleTypeDef *htim, uint32_t Channel)
{		
	uint32_t period;
	period = 84000000/(freq*(htim->Instance->PSC+1))-1;	
	if (period > 0xffff)
		period = 0xffff;
	
	htim->Instance->ARR = period;
	if (duty<-1000)
		duty = -1000;
	else if (duty > 1000)
		duty = 1000;

 if (duty > 0)	{
		HAL_GPIO_WritePin(SIGN_GPIO_Port, SIGN_Pin, GPIO_PIN_SET); // dir = 1
	}
	else 	{
		HAL_GPIO_WritePin(SIGN_GPIO_Port, SIGN_Pin, GPIO_PIN_RESET); // dir = 0
		duty = -duty;
	}
	
	duty = (htim->Instance->ARR+1)*duty/1000;
	switch(Channel){
		case 1:	
			htim->Instance->CCR1 = (uint32_t)duty;
		break;
		case 2:
			htim->Instance->CCR2 = (uint32_t)duty;
		break;
		case 3:
			htim->Instance->CCR3 = (uint32_t)duty;
		break;
		case 4:
			htim->Instance->CCR4 = (uint32_t)duty;
		break;
		default:
		break;
	}
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
