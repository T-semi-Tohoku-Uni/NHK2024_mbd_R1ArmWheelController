/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>
#include "pid.h"
#include "R1CANIDList.h"
#include "DJI_CANIDList.h"
#include "ErrorCode.h"
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
DMA_HandleTypeDef hdma_adc1;

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan3;

UART_HandleTypeDef hlpuart1;

TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */

// CAN settings
FDCAN_TxHeaderTypeDef FDCAN1_TxHeader;
FDCAN_RxHeaderTypeDef FDCAN1_RxHeader;

FDCAN_TxHeaderTypeDef FDCAN3_TxHeader;
FDCAN_RxHeaderTypeDef FDCAN3_RxHeader;

// For ADC continuous read
uint16_t arm_positions[4] = {0};

// For ARM position PID
struct PID *PID_For_ARM_POS = NULL;

/*
 * PID Parameters
 * 限界感度法で調節
 * kpc = 62 (持続振動をするときのPゲインの大きさ)
 * T_c = 0.33 (持続振動の周波数が3Hzだった)
 *
 * -> 限界感度法がクソなのでいい感じにした
 */
double P_GAIN_FOR_ARM_POS = 0.2 * 62;
double I_GAIN_FOR_ARM_POS = 0;
double D_GAIN_FOR_ARM_POS = 0.5;

// setpoint for arm
// TODO
int setpoint[4] = {
		1900,
		1900,
		1900,
		1900
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_FDCAN3_Init(void);
static void MX_TIM6_Init(void);
static void MX_FDCAN1_Init(void);
/* USER CODE BEGIN PFP */
static void ARM_Position_PID_Init(void);
static void ARM_Position_PID_Cycle(void);
static void write_error_message(uint8_t error_code);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Set timer interrupt
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	// For arm position adc
	if (htim == &htim6) {
			printf("%d\r\n", arm_positions[1]);
			ARM_Position_PID_Cycle();
	}

}

// Set Interrupt Handler for FDCAN1 (raspberrypi, other stm ..)
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
  uint8_t FDCAN1_RxData[2] = {0};

  printf("FIFO0 callback\r\n");

  // Error Handling
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == RESET) return;
  if (hfdcan != &hfdcan1) return;

  if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &FDCAN1_RxHeader, FDCAN1_RxData) != HAL_OK) {
      printf("FDCAN3 error %" PRIu32 "\r\n", hfdcan->ErrorCode); // TODO : send this error to raspberrypi ON FDCAN1
      Error_Handler();
  }

  switch(FDCAN1_RxHeader.Identifier) {
    case CANID_ARM1:
      printf("CANID_ARM %d %d\r\n", FDCAN1_RxData[0]);
      //TODO : Update ARM position setpoint
      break;
    default:
      break;
  }
}

// Set Interrupt Handler For FDCAN3 (motor at wheel and arm)
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs) {
	uint8_t FDCAN3_RxData[8];
	// Error Handling
//	printf("FIFO1 callback\r\n");
	if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) == RESET) return;
	if (hfdcan != &hfdcan3) return;

	if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &FDCAN3_RxHeader, FDCAN3_RxData) != HAL_OK) {
		printf("FDCAN3 error %" PRIu32 "\r\n", hfdcan->ErrorCode); // TODO : send this error to raspberrypi ON FDCAN1
		Error_Handler();
	}

	// TODO : Add wheel controller
//	switch(FDCAN3_RxHeader.Identifier) {
//		default:
//			printf("CAN ID %" PRIu32 "is not cached from FIFO1 callback\r\n", FDCAN3_RxHeader.Identifier);
//	}
}


/* For ARM PID */
static void ARM_Position_PID_Init(void) {

	PID_For_ARM_POS = (struct PID *)malloc(4 * sizeof(struct PID));
	if (PID_For_ARM_POS == NULL) {
	    write_error_message(MEMORY_ERROR);
			Error_Handler();
	}

	// initialize element
	for (int arm_index = 0; arm_index < 4; arm_index++ ) {
			/*
			 * 制御周期 : 1000Hz
			 * kp : 1
			 * kd : 0
			 * ki : 0
			 * setpoint : 1500
			 * -500 : integral_min
			 * 500: integral_max
			 */
			pid_init(&PID_For_ARM_POS[arm_index], 1e-3, P_GAIN_FOR_ARM_POS, D_GAIN_FOR_ARM_POS, I_GAIN_FOR_ARM_POS, 1900, -500, 500);
	}
}

static void ARM_Position_PID_Cycle(void) {
	// Automatically set adc value to DMA, so don't need to read ADC
	if (PID_For_ARM_POS == NULL) {
	    write_error_message(NULL_POINTER_ERROR);
			Error_Handler();
	}


	uint8_t pid_controller_value[8];

	// update controller output
	for (int arm_index = 0; arm_index < 4; arm_index++ ) {
			uint16_t pid_for_arm_output = (uint16_t)(-int32_t_pid_compute(&PID_For_ARM_POS[arm_index], arm_positions[arm_index]));
			pid_controller_value[arm_index*2] = pid_for_arm_output >> 8;
			pid_controller_value[arm_index*2+1] = pid_for_arm_output & 0xFF;
	}

	// write new controller value with can
	FDCAN3_TxHeader.Identifier = DJI_CANID_TX0;
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &FDCAN3_TxHeader, pid_controller_value) != HAL_OK) {
	    write_error_message(FDCAN3_ERROR);
	    Error_Handler();
	}
}

// send error message to raspberrypi
static void write_error_message(uint8_t error_code) {
  /*
   * LSB ----------------- MSB
   * | micon ID | error_code |
   */
  uint8_t error_data = (error_code << 4) | ArmWheelController;
  FDCAN1_TxHeader.Identifier = CANID_RUNTIME_ERROR;
  FDCAN1_TxHeader.DataLength = FDCAN_DLC_BYTES_1;
  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &FDCAN1_TxHeader, &error_data) != HAL_OK) {
          Error_Handler();
  }
}

int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&hlpuart1,(uint8_t *)ptr,len,8);
    return len;
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
  MX_ADC1_Init();
  MX_LPUART1_UART_Init();
  MX_FDCAN3_Init();
  MX_TIM6_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */
  // Initialize PID library
  ARM_Position_PID_Init();
  // Start ADC and save at DMA
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&arm_positions, 4);

	printf("Complete Initialize\r\n");

	// Start timer interrupt (1kHz)
	HAL_TIM_Base_Start_IT(&htim6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 4;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 15;
  hfdcan1.Init.NominalTimeSeg2 = 4;
  hfdcan1.Init.DataPrescaler = 2;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 15;
  hfdcan1.Init.DataTimeSeg2 = 4;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
  FDCAN1_TxHeader.Identifier = 0x000;
  FDCAN1_TxHeader.IdType = FDCAN_STANDARD_ID;
  FDCAN1_TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  FDCAN1_TxHeader.DataLength = FDCAN_DLC_BYTES_1;
  FDCAN1_TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  FDCAN1_TxHeader.BitRateSwitch = FDCAN_BRS_ON;
  FDCAN1_TxHeader.FDFormat = FDCAN_FD_CAN;
  FDCAN1_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  FDCAN1_TxHeader.MessageMarker = 0;

  FDCAN_FilterTypeDef FDCAN1_sFilterConfig;
  FDCAN1_sFilterConfig.IdType = FDCAN_STANDARD_ID;
  FDCAN1_sFilterConfig.FilterIndex = 0;
  FDCAN1_sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
  FDCAN1_sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  FDCAN1_sFilterConfig.FilterID1 = 0x00;
  FDCAN1_sFilterConfig.FilterID2 = 0x7ff;

  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN1_sFilterConfig) != HAL_OK) {
      Error_Handler();
  }
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) !=
      HAL_OK) {
      Error_Handler();
  }
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
      Error_Handler();
  }
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK){
      Error_Handler();
  }

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief FDCAN3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN3_Init(void)
{

  /* USER CODE BEGIN FDCAN3_Init 0 */

  /* USER CODE END FDCAN3_Init 0 */

  /* USER CODE BEGIN FDCAN3_Init 1 */

  /* USER CODE END FDCAN3_Init 1 */
  hfdcan3.Instance = FDCAN3;
  hfdcan3.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan3.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan3.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan3.Init.AutoRetransmission = DISABLE;
  hfdcan3.Init.TransmitPause = DISABLE;
  hfdcan3.Init.ProtocolException = DISABLE;
  hfdcan3.Init.NominalPrescaler = 4;
  hfdcan3.Init.NominalSyncJumpWidth = 1;
  hfdcan3.Init.NominalTimeSeg1 = 15;
  hfdcan3.Init.NominalTimeSeg2 = 4;
  hfdcan3.Init.DataPrescaler = 1;
  hfdcan3.Init.DataSyncJumpWidth = 1;
  hfdcan3.Init.DataTimeSeg1 = 1;
  hfdcan3.Init.DataTimeSeg2 = 1;
  hfdcan3.Init.StdFiltersNbr = 1;
  hfdcan3.Init.ExtFiltersNbr = 0;
  hfdcan3.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN3_Init 2 */
  // Set TXHeader
	FDCAN3_TxHeader.IdType = FDCAN_STANDARD_ID;
	FDCAN3_TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	FDCAN3_TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	FDCAN3_TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	FDCAN3_TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	FDCAN3_TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	FDCAN3_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	FDCAN3_TxHeader.MessageMarker = 0;

	// Set FDCAN3 filter config
	FDCAN_FilterTypeDef FDCAN3_sFilterConfig;
	FDCAN3_sFilterConfig.IdType = FDCAN_STANDARD_ID;
	FDCAN3_sFilterConfig.FilterIndex = 0;
	FDCAN3_sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
	FDCAN3_sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
	FDCAN3_sFilterConfig.FilterID1 = 0x000;
	FDCAN3_sFilterConfig.FilterID2 = 0x7ff;

	if (HAL_FDCAN_ConfigFilter(&hfdcan3, &FDCAN3_sFilterConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_FDCAN_Start(&hfdcan3) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK) {
		Error_Handler();
	}
  /* USER CODE END FDCAN3_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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
  htim6.Init.Prescaler = 80;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
