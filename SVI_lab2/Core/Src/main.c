/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
UART_HandleTypeDef huart1;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FMC_Init(void);
static void MX_USART1_UART_Init(void);
void ExtSramWrite(uint32_t addr, uint8_t data);
void Reset_HLx_Leds();
void Rest_all();
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Global vars for interrupt handling */
uint8_t btn1_flag = 0;
uint8_t btn2_flag = 0;
uint8_t btn3_flag = 0;
uint8_t btn4_flag = 0;
uint8_t prev_state = 0;
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
  MX_FMC_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  Reset_all();

	  if(btn1_flag == 1) {
		  btn1_flag = 0;
		  /*Mode 1*/
		  ExtSramWrite(0x60000001, 0xCC);
		  Reset_HLx_Leds();
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3|
								   GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);
		  HAL_Delay(500);

		  ExtSramWrite(0x60000001, 0x33);
		  Reset_HLx_Leds();
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|
								   GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
		  HAL_Delay(500);

		  /*Mode 2*/
		  ExtSramWrite(0x60000001, 0xF0);
		  Reset_HLx_Leds();
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|
								   GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);
		  HAL_Delay(500);
		  ExtSramWrite(0x60000001, 0xF);
		  Reset_HLx_Leds();
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|
								   GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_SET);
		  HAL_Delay(500);

	  } else if(btn2_flag == 1) {
		  btn2_flag = 0;
		  /*Mode 3*/
		  ExtSramWrite(0x60000001, 0xC3);
		  Reset_HLx_Leds();
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|
								   GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);
		  HAL_Delay(500);
		  ExtSramWrite(0x60000001, 0x3C);
		  Reset_HLx_Leds();
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3|
								   GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
		  HAL_Delay(500);

		  /*Mode 4*/
		  ExtSramWrite(0x60000001, 0xE7);
		  Reset_HLx_Leds();
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_5|
								   GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);
		  HAL_Delay(500);
		  ExtSramWrite(0x60000001, 0x18);
		  Reset_HLx_Leds();
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_SET);
		  HAL_Delay(500);

	  } else if(btn3_flag == 1) {
		  btn3_flag = 0;
		  /*Mode 5*/
		  ExtSramWrite(0x60000001, 0x66);
		  Reset_HLx_Leds();
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|
								   GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_SET);
		  HAL_Delay(500);
		  ExtSramWrite(0x60000001, 0x99);
		  Reset_HLx_Leds();
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_3|
								   GPIO_PIN_4|GPIO_PIN_7, GPIO_PIN_SET);
		  HAL_Delay(500);

		  /*Mode 6*/
		  ExtSramWrite(0x60000001, 0x7E);
		  Reset_HLx_Leds();
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|
								   GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_SET);
		  HAL_Delay(500);
		  ExtSramWrite(0x60000001, 0x81);
		  Reset_HLx_Leds();
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_7, GPIO_PIN_SET);
		  HAL_Delay(500);

	  } else if(btn4_flag == 1) {
		  btn4_flag = 0;
		  /*Mode 7*/
		  ExtSramWrite(0x60000001, 0x5A);
		  Reset_HLx_Leds();
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_3|
								   GPIO_PIN_4|GPIO_PIN_6, GPIO_PIN_SET);
		  HAL_Delay(500);
		  ExtSramWrite(0x60000001, 0xA5);
		  Reset_HLx_Leds();
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_2|
								   GPIO_PIN_5|GPIO_PIN_7, GPIO_PIN_SET);
		  HAL_Delay(500);

		  /*Mode 8*/
		  ExtSramWrite(0x60000001, 0x8E);
		  Reset_HLx_Leds();\
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|
								   GPIO_PIN_3|GPIO_PIN_7, GPIO_PIN_SET);
		  HAL_Delay(500);
		  ExtSramWrite(0x60000001, 0x71);
		  Reset_HLx_Leds();
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_4|
								   GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_SET);
		  HAL_Delay(500);
	  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_NORSRAM_TimingTypeDef Timing = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FMC_NORSRAM_DEVICE;
  hsram1.Extended = FMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FMC_NORSRAM_MEM_BUS_WIDTH_8;
  hsram1.Init.BurstAccessMode = FMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FMC_WRITE_BURST_DISABLE;
  hsram1.Init.ContinuousClock = FMC_CONTINUOUS_CLOCK_SYNC_ONLY;
  hsram1.Init.PageSize = FMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 255;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
/*
 * Write data to the address from addr-value
 */
void ExtSramWrite(uint32_t addr, uint8_t data) {
	uint8_t* pAddress = (uint8_t*)addr;
	*pAddress = data;
}

/*
 * Reset leds in line HL0..HL7 on board
 */
void Reset_HLx_Leds() {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                            |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);
}
/*
 * Reset all peripherals LEDS
 */
void Reset_all() {
	Reset_HLx_Leds();
	ExtSramWrite(0x60000001, 0x00);

}

/* Callback from buttons IRQhandler */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if(GPIO_Pin == GPIO_PIN_4) {
		//BTN1 were pressed
		btn1_flag = 1;
	} else if(GPIO_Pin == GPIO_PIN_5) {
		//BTN2 were pressed
		btn2_flag = 1;
	} else if(GPIO_Pin == GPIO_PIN_0) {
		//BTN3 were pressed
		btn3_flag = 1;
	} else if(GPIO_Pin == GPIO_PIN_1) {
		//BTN4 were pressed
		btn4_flag = 1;
	} else {
		__NOP();
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
