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
#include "stdio.h"
#include <stdlib.h>
#include "string.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_PACKET_LEN 255
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
uint16_t ADCin = 0;
uint64_t _micro = 0;
uint16_t dataOut = 0;
uint8_t DACConfig = 0b0011;


typedef struct _UartStructure
{
	UART_HandleTypeDef *huart;
	uint16_t TxLen, RxLen;
	uint8_t *TxBuffer;
	uint16_t TxTail, TxHead;
	uint8_t *RxBuffer;
	uint16_t RxTail; //RXHeadUseDMA

} UARTStucrture;

UARTStucrture UART2 = { 0 };

typedef struct _TinFGStructure
{
	uint16_t Function_GEN_Mode;
	float Function_GEN_Vpmax;
	float Function_GEN_Vpmin;
	float Function_GEN_Period_ms;
	uint16_t Function_GEN_Setting;

	float Function_GEN_Vdpp;
	float Function_GEN_Slope;
	float Function_GEN_Offset;
	float Function_GEN_Amplitude;
	float Function_GEN_Dutycycle;
	float Function_GEN_SlopeType;

} TinFGStructure;

TinFGStructure TinFG = {0};



typedef enum
{
	FG_1stHeader,
	FG_2ndHeader,
	FG_ModeSelect,

	FG_Vmax_1,
	FG_Vmax_2,
	FG_Vmax_3,
	FG_Vmax_4,

	FG_Vmin_1,
	FG_Vmin_2,
	FG_Vmin_3,
	FG_Vmin_4,

	FG_Freq_1,
	FG_Freq_2,
	FG_Freq_3,

	FG_SlopeSet,

	FG_DutyCycleSet1,
	FG_DutyCycleSet2,
	FG_DutyCycleSet3,

	FG_END,
	FG_Execute

} FGState;

FGState State = FG_1stHeader;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */
void MCP4922SetOutput(uint8_t Config, uint16_t DACOutput);
uint64_t micros();

void UARTInit(UARTStucrture *uart);
void UARTResetStart(UARTStucrture *uart);
uint32_t UARTGetRxHead(UARTStucrture *uart);
int16_t UARTReadChar(UARTStucrture *uart);
void UARTTxDumpBuffer(UARTStucrture *uart);
void UARTTxWrite(UARTStucrture *uart, uint8_t *pData, uint16_t len);
void UARTTxWrite(UARTStucrture *uart, uint8_t *pData, uint16_t len);
void TinFGProtocol(int16_t dataIn, TinFGStructure *var);
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
  MX_USART2_UART_Init();
  MX_SPI3_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_Base_Start_IT(&htim11);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &ADCin, 1);

	HAL_GPIO_WritePin(LOAD_GPIO_Port, LOAD_Pin, GPIO_PIN_RESET);

	UART2.huart = &huart2;
	UART2.RxLen = 255;
	UART2.TxLen = 255;
	UARTInit(&UART2);
	UARTResetStart(&UART2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{

		int16_t inputChar = UARTReadChar(&UART2);
		if (inputChar != -1)
		{
//			char temp[32];
//			sprintf(temp, "Recived [%d]\r\n", inputChar);
//			UARTTxWrite(&UART2, (uint8_t*) temp, strlen(temp));
			TinFGProtocol(inputChar, &TinFG);
		}


		static uint64_t timestamp = 0;
		static uint64_t timestamp2 = 0;

			if (micros() - timestamp >= 100)
			{
				timestamp = micros();
				if (TinFG.Function_GEN_Setting == 1)
				{
					timestamp2 = micros();
					TinFG.Function_GEN_Setting = 0;
				}

			if ((micros()-timestamp2) >= TinFG.Function_GEN_Period_ms)
				{
				 timestamp2 = micros();
				}

			if ( TinFG.Function_GEN_Period_ms > 0)
			{
				switch (TinFG.Function_GEN_Mode)
				{
					case 0: //DC
						dataOut = TinFG.Function_GEN_Vpmax;
					break;

					case 1: //Saw tooth
						if(TinFG.Function_GEN_SlopeType == 0)
						{
							dataOut = (TinFG.Function_GEN_Slope)*(micros()-timestamp2) + TinFG.Function_GEN_Vpmin;
						}
						else if (TinFG.Function_GEN_SlopeType == 1)
						{
							dataOut = (-1*(TinFG.Function_GEN_Slope)*(micros()-timestamp2)) + TinFG.Function_GEN_Vpmax;
						}
					break;

					case 2: //Sine wave
						dataOut =  (TinFG.Function_GEN_Amplitude*(sin((2*3.141*(micros()-timestamp2)) / TinFG.Function_GEN_Period_ms))) + TinFG.Function_GEN_Offset;
					break;

					case 3: //Square wave
						if ((micros()-timestamp2) <= ((TinFG.Function_GEN_Dutycycle*TinFG.Function_GEN_Period_ms)/100.0))
						{
							dataOut = TinFG.Function_GEN_Vpmax;
						}
						else
						{
							dataOut = TinFG.Function_GEN_Vpmin;
						}
					break;
				}
			}
			if (hspi3.State == HAL_SPI_STATE_READY && HAL_GPIO_ReadPin(SPI_SS_GPIO_Port, SPI_SS_Pin)== GPIO_PIN_SET)
			{
				MCP4922SetOutput(DACConfig, dataOut);
			}
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		UARTTxDumpBuffer(&UART2);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 100;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 99;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 65535;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_SS_GPIO_Port, SPI_SS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SHDN_GPIO_Port, SHDN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LOAD_GPIO_Port, LOAD_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin LOAD_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LOAD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_SS_Pin */
  GPIO_InitStruct.Pin = SPI_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_SS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SHDN_Pin */
  GPIO_InitStruct.Pin = SHDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SHDN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void UARTInit(UARTStucrture *uart)
{
	//dynamic memory allocate
	uart->RxBuffer = (uint8_t*) calloc(sizeof(uint8_t), UART2.RxLen);
	uart->TxBuffer = (uint8_t*) calloc(sizeof(uint8_t), UART2.TxLen);
	uart->RxTail = 0;
	uart->TxTail = 0;
	uart->TxHead = 0;

}

void UARTResetStart(UARTStucrture *uart)
{
	HAL_UART_Receive_DMA(uart->huart, uart->RxBuffer, uart->RxLen);
}

uint32_t UARTGetRxHead(UARTStucrture *uart)
{
	return uart->RxLen - __HAL_DMA_GET_COUNTER(uart->huart->hdmarx);
}

int16_t UARTReadChar(UARTStucrture *uart)
{
	int16_t Result = -1; // -1 Mean no new data

	//check Buffer Position
	if (uart->RxTail != UARTGetRxHead(uart))
	{
		//get data from buffer
		Result = uart->RxBuffer[uart->RxTail];
		uart->RxTail = (uart->RxTail + 1) % uart->RxLen;

	}
	return Result;
}

void UARTTxDumpBuffer(UARTStucrture *uart)
{
	static uint8_t MultiProcessBlocker = 0;

	if (uart->huart->gState == HAL_UART_STATE_READY && !MultiProcessBlocker)
	{
		MultiProcessBlocker = 1;

		if (uart->TxHead != uart->TxTail)
		{
			//find len of data in buffer (Circular buffer but do in one way)
			uint16_t sentingLen =
					uart->TxHead > uart->TxTail ?
							uart->TxHead - uart->TxTail :
							uart->TxLen - uart->TxTail;

			//sent data via DMA
			HAL_UART_Transmit_DMA(uart->huart, &(uart->TxBuffer[uart->TxTail]),
					sentingLen);
			//move tail to new position
			uart->TxTail = (uart->TxTail + sentingLen) % uart->TxLen;

		}
		MultiProcessBlocker = 0;
	}
}

void UARTTxWrite(UARTStucrture *uart, uint8_t *pData, uint16_t len)
{
	//check data len is more than buffur?
	uint16_t lenAddBuffer = (len <= uart->TxLen) ? len : uart->TxLen;
	// find number of data before end of ring buffer
	uint16_t numberOfdataCanCopy =
			lenAddBuffer <= uart->TxLen - uart->TxHead ?
					lenAddBuffer : uart->TxLen - uart->TxHead;
	//copy data to the buffer
	memcpy(&(uart->TxBuffer[uart->TxHead]), pData, numberOfdataCanCopy);

	//Move Head to new position

	uart->TxHead = (uart->TxHead + lenAddBuffer) % uart->TxLen;
	//Check that we copy all data That We can?
	if (lenAddBuffer != numberOfdataCanCopy)
	{
		memcpy(uart->TxBuffer, &(pData[numberOfdataCanCopy]),
				lenAddBuffer - numberOfdataCanCopy);
	}
	UARTTxDumpBuffer(uart);
}

void TinFGProtocol(int16_t dataIn, TinFGStructure *var)
	{
		//all Static Variable
		//static FGState State = FG_idle;
		static float Vmax = 0;   //mV
		static float Vmin = 0;
		static float Freq = 0;
		static float GenMode = 0;
		static float Slopemode = 0;
		static float DutyCycle = 0;
		var->Function_GEN_Setting = 0;

		switch (State)
		{
			case FG_1stHeader:
				if (dataIn == 'f')
				{State = FG_2ndHeader;}
				break;
			case FG_2ndHeader:
				if (dataIn == 'g')
				{State = FG_ModeSelect;}
				else
				{State = FG_1stHeader;}
				break;
			case FG_ModeSelect:
				if ((dataIn-48 >= 0) &&  (dataIn-48 <= 3))
				{
				GenMode = (dataIn-48);
				State = FG_Vmax_1;
				}
				else
				{State = FG_1stHeader;}
				break;
			case FG_Vmax_1:
				if ((dataIn-48 >= 0) &&  (dataIn-48 <= 9))
				{
				Vmax += (dataIn-48)*1000;
				State = FG_Vmax_2;
				}
				else
				{State = FG_1stHeader;}
				break;
			case FG_Vmax_2:
				if ((dataIn-48 >= 0) &&  (dataIn-48 <= 9))
				{
				Vmax += (dataIn-48)*100;
				State = FG_Vmax_3;
				}
				else
				{State = FG_1stHeader;}
			break;
			case FG_Vmax_3:
				if ((dataIn-48 >= 0) &&  (dataIn-48 <= 9))
				{
				Vmax += (dataIn-48)*10;
				State = FG_Vmax_4;
				}
				else
				{State = FG_1stHeader;}
				break;
			case FG_Vmax_4:
				if ((dataIn-48 >= 0) &&  (dataIn-48 <= 9))
				{
				Vmax += (dataIn-48)*1;
				State = FG_Vmin_1;
				}
				else
				{State = FG_1stHeader;}
				break;
			case FG_Vmin_1:
				if ((dataIn-48 >= 0) &&  (dataIn-48 <= 9))
				{
				Vmin += (dataIn-48)*1000;
				State = FG_Vmin_2;
				}
				else
				{State = FG_1stHeader;}
				break;
			case FG_Vmin_2:
				if ((dataIn-48 >= 0) &&  (dataIn-48 <= 9))
				{
				Vmin += (dataIn-48)*100;
				State = FG_Vmin_3;
				}
				else
				{State = FG_1stHeader;}
				break;
			case FG_Vmin_3:
				if ((dataIn-48 >= 0) &&  (dataIn-48 <= 9))
				{
				Vmin += (dataIn-48)*10;
				State = FG_Vmin_4;
				}
				else
				{State = FG_1stHeader;}
				break;
			case FG_Vmin_4:
				if ((dataIn-48 >= 0) &&  (dataIn-48 <= 9))
				{
				Vmin += (dataIn-48)*1;
				State = FG_Freq_1;
				}
				else
				{State = FG_1stHeader;}
				break;
			case FG_Freq_1:
				if ((dataIn-48 >= 0) &&  (dataIn-48 <= 9))
				{
				Freq += (dataIn-48)*10;
				State = FG_Freq_2;
				}
				else
				{State = FG_1stHeader;}
				break;
			case FG_Freq_2:
				if ((dataIn-48 >= 0) &&  (dataIn-48 <= 9))
				{
				Freq += (dataIn-48)*1;
				State = FG_Freq_3;
				}
				else
				{State = FG_1stHeader;}
				break;
			case FG_Freq_3:
				if ((dataIn-48 >= 0) &&  (dataIn-48 <= 9))
				{
				Freq += (dataIn-48)*0.1;
				State = FG_SlopeSet;
				}
				else
				{State = FG_1stHeader;}
				break;

			case FG_SlopeSet:
				if ((dataIn-48 >= 0) &&  (dataIn-48 <= 1))
				{
				Slopemode = (dataIn-48);
				State = FG_DutyCycleSet1;
				}
				else
				{State = FG_1stHeader;}
				break;

			case FG_DutyCycleSet1:
				if ((dataIn-48 >= 0) &&  (dataIn-48 <= 1))
				{
				DutyCycle += (dataIn-48)*100;
				State = FG_DutyCycleSet2;
				}
				else
				{State = FG_1stHeader;}
				break;
			case FG_DutyCycleSet2:
				if ((dataIn-48 >= 0) &&  (dataIn-48 <= 9))
				{
				DutyCycle += (dataIn-48)*10;
				State = FG_DutyCycleSet3;
				}
				else
				{State = FG_1stHeader;}
				break;
			case FG_DutyCycleSet3:
				if ((dataIn-48 >= 0) &&  (dataIn-48 <= 9))
				{
				DutyCycle += (dataIn-48)*1;
				State = FG_END;
				}
				else
				{State = FG_1stHeader;}
				break;
			case FG_END:
				if (dataIn == 'e')
				{
				State = FG_Execute;
				}
				else
				{State = FG_1stHeader;}
				break;
		}

		if (State == FG_Execute)
		{
			State = FG_1stHeader;
			///Setting
			var->Function_GEN_Mode = GenMode;
			var->Function_GEN_Vpmax = (Vmax*4096)/3300.0;
			var->Function_GEN_Vpmin = (Vmin*4096)/3300.0;
			var->Function_GEN_Period_ms = 1000000/Freq;

			var->Function_GEN_SlopeType = Slopemode;
			var->Function_GEN_Dutycycle = DutyCycle;

			var->Function_GEN_Vdpp = (var->Function_GEN_Vpmax) - (var->Function_GEN_Vpmin);
			var->Function_GEN_Slope = (var->Function_GEN_Vdpp)/(var->Function_GEN_Period_ms);
			var->Function_GEN_Offset = ((var->Function_GEN_Vpmax) + (var->Function_GEN_Vpmin))/2.0;
			var->Function_GEN_Amplitude = ((var->Function_GEN_Vpmax) - (var->Function_GEN_Vpmin))/2.0;

			var->Function_GEN_Setting = 1;


			char temp[32];
			sprintf(temp, "FFFFFFFFFF\r\n");
			UARTTxWrite(&UART2, (uint8_t*) temp, strlen(temp));

			///Reset Var
			GenMode = 0;
			Vmax = 0;
			Vmin = 0;
			DutyCycle = 0;
			Slopemode = 0;
			Freq = 0;
		}

	}



void MCP4922SetOutput(uint8_t Config, uint16_t DACOutput)
{
	uint32_t OutputPacket = (DACOutput & 0x0fff) | ((Config & 0xf) << 12);
	HAL_GPIO_WritePin(SPI_SS_GPIO_Port, SPI_SS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_IT(&hspi3, &OutputPacket, 1);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi == &hspi3)
	{
		HAL_GPIO_WritePin(SPI_SS_GPIO_Port, SPI_SS_Pin, GPIO_PIN_SET);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim11)
	{
		_micro += 65535;
	}
}

inline uint64_t micros()
{
	return htim11.Instance->CNT + _micro;
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
