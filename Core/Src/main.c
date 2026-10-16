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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dwt.h"
#include "oled.h"
#include "hcsr.h"
#include "stdio.h"
#include "motor.h"
#include "encoder.h"
#include "mpu6050.h"
#include "PID.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
unsigned char Str_Buff[64];
float distance;
uint32_t Tim2_Counter;
short encoder1;
short encoder2;
extern float Roll,Pitch,Yaw; 
uint8_t Flag_Balance=0;
uint8_t Flag_Avoid=0;
uint8_t Uart_Buff[128]={0};
uint8_t Uart_Dat;
uint8_t Forward_Flag, Backward_Flag, TurnLeft_Flag, TurnRight_Flag;


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
void OLED_Show(void);
void Scan_Key(void);
void LED_Mode(void);
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
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  DWT_Delay_Init();
  OLED_Init();
  MPU6050_initialize();
  
  if(DMP_Init()!=8) while(1);
  
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
  
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
  
  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  
  HAL_UART_Receive_IT(&huart3,Uart_Buff,1);
  //Set_PWM(5000,5000);
  
  

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  Trigger_Hcsr04();
	  OLED_Show();
	  Scan_Key();
	  
	  
	
	  LED_Mode();
	  HAL_Delay(100);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void OLED_Show()
{
	sprintf((char *)Str_Buff,"distance:%.2f ",distance);
	OLED_ShowString(0,0,Str_Buff);
	
	sprintf((char *)Str_Buff,"encoder1:%d ",encoder1);
	OLED_ShowString(0,2,Str_Buff);
	
	sprintf((char *)Str_Buff,"Pitch:%.2f ",Pitch);
	OLED_ShowString(0,4,Str_Buff);
	
	sprintf((char *)Str_Buff,"Yaw:%.2f ",Yaw);
	OLED_ShowString(0,6,Str_Buff);
	
}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin== Echo_Pin)
	{
		if(HAL_GPIO_ReadPin(Echo_GPIO_Port,Echo_Pin)==SET){
			__HAL_TIM_SetCounter(&htim2,0);
			HAL_TIM_Base_Start(&htim2);
		}
		else
		{
			HAL_TIM_Base_Stop(&htim2);
			Tim2_Counter=__HAL_TIM_GetCounter(&htim2);
			distance = Tim2_Counter * 0.017; //cm  
		}
		
     }
	else if(GPIO_Pin ==MPU6050_INT_Pin)
	{
		Control();
		static int countInco =0;
		countInco++;
		if(countInco== 100)// 0.5s
		{
			HAL_GPIO_TogglePin(PID_LED_GPIO_Port,PID_LED_Pin);
			countInco=0;
		}
		
	}
	
	
	
}

int fputc(int ch, FILE*f)
{
	HAL_UART_Transmit(&huart1,(uint8_t*)&ch,1,HAL_MAX_DELAY);
	return ch;
}


void Scan_Key()
{
	if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin) ==0)
	{
		HAL_Delay(10);
		if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin)==0)
		{
			Flag_Balance=!Flag_Balance;
			Flag_Avoid=!Flag_Avoid;
			
		}
		while(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin)==0);	
	}
}




void LED_Mode()
{
	if(Flag_Avoid==1)
	{
		HAL_GPIO_WritePin(Avoid_LED_GPIO_Port,Avoid_LED_Pin,GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(Avoid_LED_GPIO_Port,Avoid_LED_Pin,GPIO_PIN_SET);
	}
	if(Flag_Balance==1)
	{
		HAL_GPIO_WritePin(Balance_LED_GPIO_Port,Balance_LED_Pin,GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(Balance_LED_GPIO_Port,Balance_LED_Pin,GPIO_PIN_SET);
	}
	
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART3)
	{
		Uart_Dat = Uart_Buff[0];
		if(Uart_Dat == 0x5A){Forward_Flag = 0;Backward_Flag = 0;TurnLeft_Flag = 0;TurnRight_Flag = 0;}//停止 
		else if(Uart_Dat == 0x41){Forward_Flag = 1;Backward_Flag = 0;TurnLeft_Flag = 0;TurnRight_Flag = 0;}//前进A
		else if(Uart_Dat == 0x45){Forward_Flag = 0;Backward_Flag = 1;TurnLeft_Flag = 0;TurnRight_Flag = 0;}//后退E
		else if(Uart_Dat == 0x42 || Uart_Dat == 0x43 || Uart_Dat == 0x44)//右转
		{
			Forward_Flag = 0;
			Backward_Flag = 0;
			TurnLeft_Flag = 0;
			TurnRight_Flag = 1;
		}
		else if(Uart_Dat == 0x46 || Uart_Dat == 0x47 || Uart_Dat == 0x48)//左转
		{
			Forward_Flag = 0;
			Backward_Flag = 0;
			TurnLeft_Flag = 1;
			TurnRight_Flag = 0;
		}
		else{Forward_Flag = 0;Backward_Flag = 0;TurnLeft_Flag = 0;TurnRight_Flag = 0;}//停止
		
		HAL_UART_Receive_IT(&huart3,Uart_Buff,1);
		
		
	}
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
