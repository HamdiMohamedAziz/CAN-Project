/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MQ2.h"
#include <math.h>
#include "DHT.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

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

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_ch1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char uartBufR[64];
char txBuf[200];	// buffer for serial communication
uint8_t adc_valid = 0;
/* ADC Samples array */
uint32_t NUM_SAMPLES = 0;
uint32_t i = 0;
uint32_t ADC_SAMPLES[1000];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void usDelay(uint16_t us);
void DelayTIM4_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

uint8_t TxData[8];
uint8_t RxData[8];

uint32_t TxMailbox;
int datacheck=0;
ADC_HandleTypeDef hadc1;
UART_HandleTypeDef huart2;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	adc_valid = 1;
	if(i < NUM_SAMPLES){
		ADC_SAMPLES[i] = HAL_ADC_GetValue(&hadc1);
		i++;
		HAL_ADC_Start_IT(&hadc1);
	}
	else{
		i = 0;	// reinitialize iterator
		HAL_ADC_Stop_IT(&hadc1);
	}
}

/* MQ2 Sensor Specific Functions */
void MQ2_create(MQ2 *sensor) {
    sensor->LPGCurve[0] = 2.3;
    sensor->LPGCurve[1] = 0.21;
    sensor->LPGCurve[2] = -0.47;
    sensor->COCurve[0] = 2.3;
    sensor->COCurve[1] = 0.72;
    sensor->COCurve[2] = -0.34;
    sensor->SmokeCurve[0] = 2.3;
    sensor->SmokeCurve[1] = 0.53;
    sensor->SmokeCurve[2] = -0.44;
    sensor->Ro = -1.0;
    sensor->lastReadTime = 0;
}

void MQ2_begin(MQ2 *sensor) {
    // Initialization code here
	sensor->Ro = MQ2_MQCalibration(sensor);

	sprintf(txBuf, "Ro: %f kohm\r\n\r\n", sensor->Ro);
	HAL_UART_Transmit_IT(&huart2, (uint8_t *)txBuf, strlen(txBuf));
	HAL_Delay(50);
}

void MQ2_close(MQ2 *sensor) {
	sensor->Ro = -1.0;
	sensor->values[0] = 0.0;
	sensor->values[1] = 0.0;
	sensor->values[2] = 0.0;
}

bool MQ2_checkCalibration(MQ2 *sensor) {
	if (sensor->Ro < 0.0) {
		sprintf(txBuf, "%s ", "Device not calibrated, call MQ2_begin before reading any value.");
		HAL_UART_Transmit_IT(&huart2, (uint8_t *)txBuf, strlen(txBuf));
		return false;
	}
	return true;
}

float* MQ2_read(MQ2 *sensor, bool print) {
    // Reading code here
	if (!MQ2_checkCalibration(sensor)) return NULL;

	sensor->values[0] = MQ2_MQGetPercentage(sensor->LPGCurve, sensor);
	sensor->values[1] = MQ2_MQGetPercentage(sensor->COCurve, sensor);
	sensor->values[2] = MQ2_MQGetPercentage(sensor->SmokeCurve, sensor);

	sensor->lastReadTime = HAL_GetTick();

	if (print){
		sprintf(txBuf, "%dms\r\n", sensor->lastReadTime);
		HAL_UART_Transmit_IT(&huart2, (uint8_t *)txBuf, strlen(txBuf));
		HAL_Delay(50);

		sprintf(txBuf, "LPG: %f ppm\r\n", sensor->values[0]);
		HAL_UART_Transmit_IT(&huart2, (uint8_t *)txBuf, strlen(txBuf));
		HAL_Delay(50);

		sprintf(txBuf, "CO: %f ppm \r\n", sensor->values[1]);
		HAL_UART_Transmit_IT(&huart2, (uint8_t *)txBuf, strlen(txBuf));
		HAL_Delay(50);

		sprintf(txBuf, "SMOKE: %f ppm\r\n", sensor->values[2]);
		HAL_UART_Transmit_IT(&huart2, (uint8_t *)txBuf, strlen(txBuf));
		HAL_Delay(50);
	}

	return sensor->values;
}

float MQ2_readLPG(MQ2 *sensor) {

	if (!MQ2_checkCalibration(sensor)) return 0.0;

	if (HAL_GetTick() < (sensor->lastReadTime + READ_DELAY) && sensor->values[0] > 0)
		return sensor->values[0];
	else
		return (sensor->values[0] = MQ2_MQGetPercentage(sensor->LPGCurve, sensor));
}

float MQ2_readCO(MQ2 *sensor) {
	if (!MQ2_checkCalibration(sensor)) return 0.0;

	if (HAL_GetTick() < (sensor->lastReadTime + READ_DELAY) && sensor->values[1] > 0)
		return sensor->values[1];
	else
		return (sensor->values[1] = MQ2_MQGetPercentage(sensor->COCurve, sensor));
}

float MQ2_readSmoke(MQ2 *sensor) {
	if (!MQ2_checkCalibration(sensor)) return 0.0;

	if (HAL_GetTick() < (sensor->lastReadTime + READ_DELAY) && sensor->values[2] > 0)
		return sensor->values[2];
	else
		return (sensor->values[2] = MQ2_MQGetPercentage(sensor->SmokeCurve, sensor));
}

float MQ2_MQResistanceCalculation(int raw_adc) {
    // MQResistanceCalculation code here
	float flt_adc = (float) raw_adc;
	return RL_VALUE * (1023.0 - flt_adc) / flt_adc;
}

float MQ2_MQCalibration(MQ2 *sensor) {
    // MQCalibration code here
	float val = 0.0;

	// take multiple samples
	NUM_SAMPLES = CALIBARAION_SAMPLE_TIMES;
	HAL_ADC_Start_IT(&hadc1);
	HAL_Delay(CALIBRATION_SAMPLE_INTERVAL);

	for (int i = 0; i < CALIBARAION_SAMPLE_TIMES; i++) {
		val += MQ2_MQResistanceCalculation(ADC_SAMPLES[i]);
	}

	//calculate the average value
	val = val / ((float) CALIBARAION_SAMPLE_TIMES);

	//divided by RO_CLEAN_AIR_FACTOR yields the Ro according to the chart in the datasheet
	val = val / RO_CLEAN_AIR_FACTOR;

	return val;
}

float MQ2_MQRead(MQ2 *sensor) {
	float rs = 0.0;

	// take multiple samples
	NUM_SAMPLES = READ_SAMPLE_TIMES;
	HAL_ADC_Start_IT(&hadc1);
	HAL_Delay(READ_SAMPLE_INTERVAL);

	for (int i = 0; i < READ_SAMPLE_TIMES; i++) {
		rs += MQ2_MQResistanceCalculation(ADC_SAMPLES[i]);
	}

	return rs / ((float) READ_SAMPLE_TIMES);  // return the average
}

float MQ2_MQGetPercentage(float *pcurve, MQ2 *sensor) {
	float rs_ro_ratio = MQ2_MQRead(sensor) / sensor->Ro;
	return pow(10.0, ((log(rs_ro_ratio) - pcurve[1]) / pcurve[2]) + pcurve[0]);
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==GPIO_PIN_13)
	{
		TxData[0]=0; //ms delay
		TxData[1]=0;  //loop
		TxData[2]=0;

		HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, TxMailbox);
	}
	}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
	if(RxHeader.DLC==2)
	{
		datacheck=1;
	}
	}

char uartBuf[100];

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
//	uint32_t numTicks = 0;////////////////////////////////////////

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
  MX_CAN1_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
HAL_CAN_Start(&hcan1);
// Setup
MQ2 sensor;
MQ2_create(&sensor);
MQ2_begin(&sensor);
float lpg, co, smoke;
//DelayTIM4_Init();

static DHT_sensor livingRoom = {GPIOB, GPIO_PIN_4, DHT11, GPIO_NOPULL};
//activate notification
HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

TxHeader.DLC=3;
TxHeader.IDE=CAN_ID_STD;
TxHeader.RTR=CAN_RTR_DATA;
TxHeader.StdId=0x446;
//Ro = MQ4_CalculateRo();

HAL_Delay(2000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  MQ2_read(&sensor, true); //set it false if you don't want to print the values to the Serial

	 	  // lpg = values[0];
	 	  lpg = MQ2_readLPG(&sensor);
	 	  // co = values[1];
	 	  co = MQ2_readCO(&sensor);
	 	  // smoke = values[2];
	 	  smoke = MQ2_readSmoke(&sensor);

	 	  sprintf(txBuf, "lpg: %f\r\n", lpg);
	 	  HAL_UART_Transmit_IT(&huart2, (uint8_t *)txBuf, strlen(txBuf));
	 	  HAL_Delay(50);

	 	  sprintf(txBuf, "co: %f\r\n", co);
	 	  HAL_UART_Transmit_IT(&huart2, (uint8_t *)txBuf, strlen(txBuf));
	 	  HAL_Delay(50);

	 	  sprintf(txBuf, "smoke: %f\r\n\r\n", smoke);
	 	  HAL_UART_Transmit_IT(&huart2, (uint8_t *)txBuf, strlen(txBuf));
	 	  HAL_Delay(50);

	 	  HAL_Delay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  char msg[40];
	  DHT_data d = DHT_getData(&livingRoom);
	  sprintf(msg, "\f Temp %d°C, Hum %d%%", (uint8_t)d.temp, (uint8_t)d.hum);
	  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 10);////////////////////////////////////HAL_MAX_DELAY
HAL_Delay(1000);

	 // HAL_Delay(50);

	 // sprintf(txBuf, "co: %f\r\n", co);
	//  HAL_UART_Transmit_IT(&huart2, (uint8_t *)txBuf, strlen(txBuf));
	 // HAL_Delay(50);

	 // sprintf(txBuf, "smoke: %f\r\n\r\n", smoke);
	 // HAL_UART_Transmit_IT(&huart2, (uint8_t *)txBuf, strlen(txBuf));
	 // HAL_Delay(50);

	  //HAL_Delay(500);
	  //blink the led
	  if (isnan(lpg)&&(d.temp<50)) {

		  //int ValTemp = (int)d.temp;
		  TxData[0]=1; //ms delay
		  TxData[1]=0;  //loop
		  TxData[2]=d.temp;s
		  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, TxMailbox);

	  }else if(!isnan(lpg)&&(d.temp>50)){

		  int ValTemp = (int)d.temp;
		  TxData[0]=0; //ms delay
		  TxData[1]=1;  //loop
		  TxData[2]=d.temp;
		  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, TxMailbox);

	  }else if(isnan(lpg)&&(d.temp>50)){
		  int ValTemp = (int)d.temp;
		  TxData[0]=1; //ms delay
		  TxData[1]=1;  //loop
		  TxData[2]=d.temp;
		  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, TxMailbox);

	  }else{
		  int ValTemp = d.temp;
		  TxData[0]=0; //ms delay
		  TxData[1]=0;  //loop
		  TxData[2]=d.temp;
		  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, TxMailbox);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
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
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 18;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  CAN_FilterTypeDef canfilterconfig;
  canfilterconfig.FilterActivation=CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank=18;
  canfilterconfig.FilterFIFOAssignment=CAN_FILTER_FIFO0;
  canfilterconfig.FilterIdHigh=0x103<<5;
  canfilterconfig.FilterIdLow=0;
  canfilterconfig.FilterMaskIdHigh=0x100<<5;
  canfilterconfig.FilterMaskIdLow=0x0000;
  canfilterconfig.FilterMode=CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale=CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank=20;

  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
