/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
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
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac_ch1;

SD_HandleTypeDef hsd;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
#define HALF_DATASIZE	512
#define FULL_DATASIZE 1024
	FATFS myFATFS;
	FIL myFILE;
volatile FRESULT fresult;
static volatile uint8_t* INPUT_DATA;
// Motion
static volatile uint8_t Motion = 0;
static volatile uint8_t Pre_Motion = 0;
// Speaking
static volatile uint8_t Speaking = 0;
// Dark mode
static volatile uint16_t ADC_value = 0;
static volatile uint8_t Dark_mode = 0;
	//	Status variable
static volatile	uint8_t MODE = 0;//  0: Stand by		1: Audio
static volatile	uint8_t UPPERCplt = 0;
static volatile	uint8_t LOWERCplt = 0;
static volatile	uint8_t ENDFILE = 0;
	// DAC
static volatile	uint8_t valByte; // DAC set value
static volatile	uint8_t DAC_VALUE[FULL_DATASIZE];
	// SD card
	UINT	writeByte;
	UINT	readByte;
static volatile uint32_t Size_of_File;
static volatile uint32_t Read_Counter;
static volatile uint32_t Transfer_Counter;
static volatile uint32_t Read_Pointer;
static volatile uint32_t R;
	char file1Path[] = "soobin.wav";
static volatile	uint8_t buffer[100];
static volatile	uint8_t buffer[100];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
  #define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif /* __GNUC__ */
	PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
	HAL_UART_Transmit(&huart3, (uint8_t*)&ch,1,100);

  /* Loop until the end of transmission */


  return ch;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Transfer UPPER buff complete
void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef* hdac)
{
		INPUT_DATA = &DAC_VALUE[0];
		UPPERCplt = 1;
		Transfer_Counter--;
		if(Transfer_Counter == 0) MODE = 0;
}
// Transfer LOWER buff complete
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac)
{
		INPUT_DATA = &DAC_VALUE[HALF_DATASIZE];
		LOWERCplt = 1;
		Transfer_Counter--;
		if(Transfer_Counter == 0) MODE = 0;
}

void	Process_Data(void)
{
			if ( UPPERCplt == 1 &&	Read_Counter)
			{
				Read_Pointer += HALF_DATASIZE ;
				f_lseek( &myFILE, Read_Pointer);
				if( Read_Counter == 1)	
					{f_read( &myFILE,(uint8_t*) INPUT_DATA , R , &readByte);}
				else 
					{f_read( &myFILE,(uint8_t*) INPUT_DATA , HALF_DATASIZE, &readByte);}
				Read_Counter--;
				UPPERCplt = 0;
				
			}
			else if( LOWERCplt == 1 && Read_Counter)
			{
				Read_Pointer += HALF_DATASIZE ;
				f_lseek( &myFILE, Read_Pointer);		
				if( Read_Counter == 1)	
					{f_read( &myFILE,(uint8_t*) INPUT_DATA , R , &readByte);}
				else 
					{f_read( &myFILE,(uint8_t*) INPUT_DATA , HALF_DATASIZE, &readByte);}
				Read_Counter--;
				LOWERCplt = 0;
				
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
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_DAC_Init();
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

	HAL_GPIO_WritePin(output_GPIO_Port, output_Pin, GPIO_PIN_SET);
	HAL_ADC_Start( &hadc1);
		/* Read Data from SD to FULL  DAC Buffer*/
			fresult = f_mount( &myFATFS,"", 1);
			if( fresult == FR_OK)
			{
				printf("MOUTN SD OK\r\n");
			}	
			fresult = f_open( &myFILE, file1Path, FA_READ);
						if( fresult == FR_OK)
					{
						printf("OPEN FILE OK\r\n");
						Size_of_File = f_size( &myFILE) - 16;
						Read_Pointer = 16;
						f_lseek( &myFILE, Read_Pointer);
						Read_Counter = Size_of_File/HALF_DATASIZE + 1;
						R = Size_of_File - Size_of_File/HALF_DATASIZE*HALF_DATASIZE;
						Transfer_Counter = Read_Counter;
						printf("File name: %s\r\n", file1Path);
						printf("Size of file: %d  Byte\r\n", Size_of_File);
						printf("R: %d  Byte\r\n", R);
						printf("Number of Block: %d  Block\r\n\r\n", Read_Counter);
						
						INPUT_DATA = &DAC_VALUE[0];
						f_read( &myFILE,(uint8_t*) INPUT_DATA , HALF_DATASIZE, &readByte);
						Read_Counter--;
						
						Read_Pointer += HALF_DATASIZE ;
						f_lseek( &myFILE, Read_Pointer);
						INPUT_DATA = &DAC_VALUE[HALF_DATASIZE];
						f_read( &myFILE,(uint8_t*) INPUT_DATA , HALF_DATASIZE, &readByte);
						Read_Counter--;
								/* Start DMA & DAC - Audio */
						MODE = 1;
						UPPERCplt = 0;
						LOWERCplt = 0;
						HAL_TIM_Base_Start(&htim6); // Start DMA Timer Trigger 
						HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*) DAC_VALUE, FULL_DATASIZE, DAC_ALIGN_8B_R); 
					}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */
	
    /* USER CODE BEGIN 3 */
		Pre_Motion = Motion;
		Motion = HAL_GPIO_ReadPin(sensor_motion_GPIO_Port, sensor_motion_Pin);

		ADC_value = HAL_ADC_GetValue( &hadc1);
		
		
		if( Motion == 1 && Pre_Motion == 0 && MODE == 0)
		{
				Speaking = 1;
			if( ADC_value > 2500)
			{
				Dark_mode = 1;
				HAL_GPIO_WritePin(output_GPIO_Port, output_Pin, GPIO_PIN_RESET); // Turm on the light
			}
		}
		if( Motion == 0 && Pre_Motion == 1 && Dark_mode == 1)
		{
				Dark_mode = 0;
				HAL_GPIO_WritePin(output_GPIO_Port, output_Pin, GPIO_PIN_SET);
		}
		
		if( Speaking)
		{
						fresult = f_open( &myFILE, file1Path, FA_READ);
						if( fresult == FR_OK)
					{
						printf("OPEN FILE OK\r\n");
						Size_of_File = f_size( &myFILE) - 16;
						Read_Pointer = 16;
						f_lseek( &myFILE, Read_Pointer);
						Read_Counter = Size_of_File/HALF_DATASIZE + 1;
						R = Size_of_File - Size_of_File/HALF_DATASIZE*HALF_DATASIZE;
						Transfer_Counter = Read_Counter;
						printf("File name: %s\r\n", file1Path);
						printf("Size of file: %d  Byte\r\n", Size_of_File);
						printf("R: %d  Byte\r\n", R);
						printf("Number of Block: %d  Block\r\n\r\n", Read_Counter);
						
						INPUT_DATA = &DAC_VALUE[0];
						f_read( &myFILE,(uint8_t*) INPUT_DATA , HALF_DATASIZE, &readByte);
						Read_Counter--;
						
						Read_Pointer += HALF_DATASIZE ;
						f_lseek( &myFILE, Read_Pointer);
						INPUT_DATA = &DAC_VALUE[HALF_DATASIZE];
						f_read( &myFILE,(uint8_t*) INPUT_DATA , HALF_DATASIZE, &readByte);
						Read_Counter--;
								/* Start DMA & DAC - Audio */
						MODE = 1;
						UPPERCplt = 0;
						LOWERCplt = 0;
						HAL_TIM_Base_Start(&htim6); // Start DMA Timer Trigger 
						HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*) DAC_VALUE, FULL_DATASIZE, DAC_ALIGN_8B_R); 
						Speaking = 0;
					}
		}
		if( MODE == 1)
		{
			Process_Data();
		}
		else
		{
			HAL_DAC_Stop_DMA( &hdac, DAC_CHANNEL_1); // Stop DMA --- Manitude
			HAL_TIM_Base_Stop( &htim6); // Stop TIMER --- Frequency
			f_close( &myFILE); // Close Song file
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization 
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config 
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 3;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 815;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(output_GPIO_Port, output_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : output_Pin */
  GPIO_InitStruct.Pin = output_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(output_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : sensor_motion_Pin */
  GPIO_InitStruct.Pin = sensor_motion_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(sensor_motion_GPIO_Port, &GPIO_InitStruct);

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
