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
#include <stdbool.h>
#include "pid.h"
#include "R1CANIDList.h"
#include "DJI_CANIDList.h"
#include "ErrorCode.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct MotorState {
  int16_t vel;
  double pos;
} MotorState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define REDUCTIONRATIO 36
#define GEARNUM 32
#define RACKPITCH 3.14159265
#define MOTOR_TO_EDGE_DISTANCE 21
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan3;

IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef hlpuart1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */

// CAN settings
FDCAN_TxHeaderTypeDef FDCAN1_TxHeader;
FDCAN_RxHeaderTypeDef FDCAN1_RxHeader;

FDCAN_TxHeaderTypeDef FDCAN3_TxHeader;
FDCAN_RxHeaderTypeDef FDCAN3_RxHeader;

// For ADC continuous read
double arm_positions[4] = {0};
MotorState arm_motor[4];

// スイッチのチャタリング防止用変数
bool isHolPushedFlagForPreventChattering[4] = {
    false,
    false,
    false,
    false
};

// チャタリング防止用のタイマー
// 1MHzでタイマー回して1000になったらリセットする
unsigned int holdTimer[4] = {
    0,
    0,
    0,
    0
};

// それぞれのスイッチのロボットからみた位置
double ArmInitializeSwitchPosition[4] = {
    -(310 - MOTOR_TO_EDGE_DISTANCE),
    -(12.5 + MOTOR_TO_EDGE_DISTANCE),
    12.5 + MOTOR_TO_EDGE_DISTANCE,
    310 - MOTOR_TO_EDGE_DISTANCE,
};
// 全てのアーム位置を再初期化する際に使う変数たち
bool isPushedRestHomePositionButton[4] = {
    false,
    false,
    false,
    false
};

// For ARM position PID
// Reset以下のプログラムを有効化にするかどうか. デフォルトはfalse, アームの制御を入れる時にtrueにする
bool isProgramRun = false;

struct PID *PID_For_ARM_POS = NULL;

/*
 * PID Parameters
 * 限界感度法で調節
 * kpc = 62 (持続振動をするときのPゲインの大きさ)
 * T_c = 0.33 (持続振動の周波数が3Hzだった)
 *
 * -> 限界感度法がクソなのでいい感じにした
 */
double P_GAIN_FOR_ARM_POS = 5;
double P_GAIN_FOR_ARM_POS_SEQ[4] = {12.0, 12.0, 12.0, 12.0};
double I_GAIN_FOR_ARM_POS = 0;
double D_GAIN_FOR_ARM_POS = 1;

// setpoint for arm
// TODO
//int setpoint[4] = {
//		2640,
//		1550,
//		2480,
//		1387,
//};
int setpoint[3][4] = {
    // 苗の回収位置
    {
        -285,
        -69.5,
        69.5,
        285,
    },
    // 外側をおく
    {
        -160,
        -50,
        50,
        160,
    },
    // 内側をおく
    {
        -285,
        -194.5,
        194.5,
        285
    }
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_FDCAN3_Init(void);
static void MX_TIM6_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM16_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */
static void ARM_Position_PID_Init(void);
static void ARM_Position_PID_Cycle(void);

// 原点調節用の関数
static void InitMotorState(uint8_t motorID);
static void setMotorVel();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* --- アームの原点を取るための関数たち --- */
void ResetToHomePosition() {
  // 実行前の初期化
  // PIDの制御を一旦止める
  HAL_TIM_Base_Stop_IT(&htim6);
  // MotorStateの初期化
  for (int arm_index = 0; arm_index < 4; arm_index++ ) {
      arm_motor[arm_index].pos = 0;
      arm_motor[arm_index].vel = 0;
  }
  // それぞれのスイッチの変数を全てfalseにする
  for (int arm_index = 0; arm_index < 4; arm_index++ ) {
      isPushedRestHomePositionButton[arm_index] = false;
  }

  /*
     * CANID 0, 3はプラス方向
     * CANID 1, 2はマイナス方向へ
  */
//  int16_t val_settings[4] = {
//      0,
//      0,
//      0,
//      -1000,
//  };
//  uint8_t motor_vel_value[8];
//
//  // update controller output
//  for (int arm_index = 0; arm_index < 4; arm_index++ ) {
//      motor_vel_value[arm_index*2] = val_settings[arm_index] >> 8;
//      motor_vel_value[arm_index*2+1] = val_settings[arm_index] & 0xFF;
//  }

  /*
   * アームをリミットスイッチまで動かす
   * スイッチが押されたら停止する
   * タイマーの切り忘れがないように注意
   */
  HAL_TIM_Base_Start_IT(&htim7);

  // 全部がスイッチにタッチするまで待つ
  printf("while\r\n");
  while (
      !isPushedRestHomePositionButton[0] ||
      !isPushedRestHomePositionButton[1] ||
      !isPushedRestHomePositionButton[2] ||
      !isPushedRestHomePositionButton[3]
  ) {

  }

  printf("Complete\r\n");

  // PIDの制御を再開
  // 原点初期化用のタイマーを停止して、PIDを再開する
  HAL_TIM_Base_Stop_IT(&htim7);
  HAL_TIM_Base_Start_IT(&htim6);
}

/*
 * 原点調節用のスイッチの割り込み関数
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == Arm0Switch_Pin && !isHolPushedFlagForPreventChattering[0])
  {
      printf("Arm 0 initialized\r\n");
      InitMotorState(0); // motorStateを再初期化する
      setMotorVel(); // 再初期化したモーターの速度を0にする（InitMotorStateで変更したvelがCANに流されて反映される）
      isPushedRestHomePositionButton[0] = true;
      isHolPushedFlagForPreventChattering[0] = true; // チャタリング防止用
  }

  if (GPIO_Pin == Arm1Switch_Pin && !isHolPushedFlagForPreventChattering[1])
  {
//      printf("[Initialize Position]: ARM 1\r\n");
      printf("Arm 1 initialized\r\n");
      InitMotorState(1); // motorStateを再初期化する
      setMotorVel(); // 再初期化したモーターの速度を0にする（InitMotorStateで変更したvelがCANに流されて反映される）
      isPushedRestHomePositionButton[1] = true;
      isHolPushedFlagForPreventChattering[1] = true; // チャタリング防止用
  }

  if (GPIO_Pin == Arm2Switch_Pin && !isHolPushedFlagForPreventChattering[2])
  {
//      printf("[Initialize Position]: ARM 4\r\n");
      printf("Arm 2 initialized\r\n");
      InitMotorState(2); // motorStateを再初期化する
      setMotorVel(); // 再初期化したモーターの速度を0にする（InitMotorStateで変更したvelがCANに流されて反映される）
      isPushedRestHomePositionButton[2] = true;
      isHolPushedFlagForPreventChattering[2] = true; // チャタリング防止用
  }

  if (GPIO_Pin == Arm3Switch_Pin && !isHolPushedFlagForPreventChattering[3])
  {
//      printf("[Initialize Position]: ARM 4\r\n");
      printf("Arm 3 initialized\r\n");
      InitMotorState(3); // motorStateを再初期化する
      setMotorVel(); // 再初期化したモーターの速度を0にする（InitMotorStateで変更したvelがCANに流されて反映される）
      isPushedRestHomePositionButton[3] = true;
      isHolPushedFlagForPreventChattering[3] = true; // チャタリング防止用
  }
}

/*
 * CANIDがmotorIDのモータのarm_positions(アームの位置)情報を初期化する
 */
void InitMotorState(uint8_t motorID) {
  arm_motor[motorID].vel = 0;
  arm_motor[motorID].pos = ArmInitializeSwitchPosition[motorID];
}

/*
 * 原点が押されるまでモーターを原点方向に回し続ける関数.
 * 原点到着後はタイマーが止まるまでその場に居続ける
 */
void MoveToOriginAndHold(void) {
//  int16_t vel_settings[4] = {0, 0, 0, 0};
//
  if (!isPushedRestHomePositionButton[0]) {
        arm_motor[0].vel = -500;
  }
  if (!isPushedRestHomePositionButton[1]) {
      arm_motor[1].vel =  500;
  }
  if (!isPushedRestHomePositionButton[2]) {
      arm_motor[2].vel = -500;
  }
  if (!isPushedRestHomePositionButton[3]) {
      arm_motor[3].vel = 500;
  }

  setMotorVel();
}

void setMotorVel() {
    uint8_t motor_vel_value[8];

    // update controller output
    for (int arm_index = 0; arm_index < 4; arm_index++ ) {
        motor_vel_value[arm_index*2] = arm_motor[arm_index].vel >> 8;
        motor_vel_value[arm_index*2+1] = arm_motor[arm_index].vel & 0xFF;
    }

    FDCAN3_TxHeader.Identifier = DJI_CANID_TX0;
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &FDCAN3_TxHeader, motor_vel_value) != HAL_OK) {
        Error_Handler();
    }
}

// Set timer interrupt
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	// For arm position adc
	if (htim == &htim6) {
	    // TODO: enable this func to enable PID
			ARM_Position_PID_Cycle();
	}
	// アームの原点調節をするときに使用する. それ以外はdisable
	if (htim == &htim7) {
	    MoveToOriginAndHold();
	}

	// チャタリング防止用. 100KHzで回している。10msなったら解除
	if (htim == &htim16) {
	    for (int arm_index = 0; arm_index < 4; arm_index++ ) {
	        if (!isHolPushedFlagForPreventChattering[arm_index]) continue;
	        holdTimer[arm_index]++;
	        // 10ms立ったら再度初期化
	        if (holdTimer[arm_index] == 1000) {
	            holdTimer[arm_index] = 0;
	            isHolPushedFlagForPreventChattering[arm_index] = false;
	        }
//	        printf("%d\r\n", isHolPushedFlagForPreventChattering[arm_index]);
	    }
	}
}

// Set Interrupt Handler for FDCAN1 (raspberrypi, other stm ..)
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
  uint8_t FDCAN1_RxData[2] = {0};

  // Error Handling
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == RESET) return;
  if (hfdcan != &hfdcan1) return;

  if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &FDCAN1_RxHeader, FDCAN1_RxData) != HAL_OK) {
      printf("FDCAN3 error %" PRIu32 "\r\n", hfdcan->ErrorCode); // TODO : send this error to raspberrypi ON FDCAN1
      Error_Handler();
  }

  if (FDCAN1_RxHeader.Identifier == CANID_SEEDLING_SET_ARM_POSITION) {
      // プログラムを有効化する
      isProgramRun = true;
      switch(FDCAN1_RxData[0]) {
        case 0:
          printf("PICKUP\r\n");
          for(int arm_index=0; arm_index < 4; arm_index++ ) {
              pid_reset_setpoint(&PID_For_ARM_POS[arm_index], setpoint[0][arm_index]);
          }
          break;
        case 1:
          printf("PUT OUTSIDE\r\n");
          for(int arm_index=0; arm_index < 4; arm_index++ ) {
              pid_reset_setpoint(&PID_For_ARM_POS[arm_index], setpoint[1][arm_index]);
          }
          break;
        case 2:
          printf("PUT INSIDE\r\n");
          for(int arm_index=0; arm_index < 4; arm_index++ ) {
              pid_reset_setpoint(&PID_For_ARM_POS[arm_index], setpoint[2][arm_index]);
          }
          break;
        case 3:
          // TODO: disable FDCAN3 and reset program
          printf("RESET\r\n");
          HAL_NVIC_DisableIRQ(FDCAN3_IT1_IRQn);
          break;
        default:
          break; // TODO : send RuntimeError to raspbeerypi
      }
  }
}

float rpm_to_signed(uint16_t angular_velocity) {
  if (angular_velocity <= UINT16_MAX/2) {
      return (float)(angular_velocity);
  } else {
      return (float)(angular_velocity - UINT16_MAX);
  }
}

int to_mechanical_angle(uint16_t angle) {
  return (int)((angle / 8191.0) * 360);
}

// Set Interrupt Handler For FDCAN3 (motor at wheel and arm)
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs) {
	uint8_t FDCAN3_RxData[8];
	float rpm;
	uint8_t motorID;

	// Error Handling
	if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) == RESET) return;
	if (hfdcan != &hfdcan3) return;

	if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &FDCAN3_RxHeader, FDCAN3_RxData) != HAL_OK) {
		printf("FDCAN3 error %" PRIu32 "\r\n", hfdcan->ErrorCode); // TODO : send this error to raspberrypi ON FDCAN1
		Error_Handler();
	}

	// Reload IWDG
	if (HAL_IWDG_Refresh(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }

//	printf("%d\r\n", motorID);
	motorID = FDCAN3_RxHeader.Identifier - DJI_CANID_TX0 - 1;
	// uint16_t 0 ~ 65535
	rpm = rpm_to_signed(FDCAN3_RxData[2] << 8 | FDCAN3_RxData[3]);
	double motor_vel = (double)(rpm / 60 / REDUCTIONRATIO * GEARNUM * RACKPITCH);
	arm_motor[motorID].vel = motor_vel;
	arm_motor[motorID].pos += (motor_vel * 0.001);
//	arm_positions[motorID] = arm_positions[motorID] + motor_vel * 0.001;

//	printf("%f, %f, %f, %f\r\n", 0.0, arm_motor[1].pos, 12.5 , 2000.0);
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
			pid_init(&PID_For_ARM_POS[arm_index], 1e-3, P_GAIN_FOR_ARM_POS_SEQ[arm_index], D_GAIN_FOR_ARM_POS, I_GAIN_FOR_ARM_POS, setpoint[0][arm_index], -500, 500);
	}
}

static void ARM_Position_PID_Cycle(void) {
	// Automatically set adc value to DMA, so don't need to read ADC
	if (PID_For_ARM_POS == NULL) {
			Error_Handler();
	}


//	uint8_t pid_controller_value[8];

//	printf("0, %d, 3000\r\n", arm_positions[1]);
//	printf("0, %d, 3000\r\n", arm_positions[2]);
//	printf("0, %d, %d, %d, %d, 3000\r\n", arm_positions[0], arm_positions[1], arm_positions[2], arm_positions[3]);
//	printf("0, %d, 3000\r\n", arm_positions[2]);

//	uint16_t arm[4] = {
//	    arm_positions[3],
//	    arm_positions[0],
//	    arm_positions[1],
//	    arm_positions[2]
//	};

	// update controller output
	for (int arm_index = 0; arm_index < 4; arm_index++ ) {
	    int32_t pid_val = int32_t_pid_compute(&PID_For_ARM_POS[arm_index], arm_motor[arm_index].pos);
			arm_motor[arm_index].vel = pid_val;
//			pid_controller_value[arm_index*2] = pid_for_arm_output >> 8;
//			pid_controller_value[arm_index*2+1] = pid_for_arm_output & 0xFF;
	}
	setMotorVel();
////	 write new controller value with can
//	FDCAN3_TxHeader.Identifier = DJI_CANID_TX0;
//	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &FDCAN3_TxHeader, pid_controller_value) != HAL_OK) {
//	    Error_Handler();
//	}
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
  MX_LPUART1_UART_Init();
  MX_FDCAN3_Init();
  MX_TIM6_Init();
  MX_FDCAN1_Init();
  MX_TIM7_Init();
  MX_TIM16_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
//  printf("arm position pid init\r\n");

  // Initialize PID library
  HAL_TIM_Base_Start_IT(&htim16);
  ARM_Position_PID_Init();

  while(!isProgramRun) {
      // アームの制御が入るまではこれ以降の処理をブロックする
      // 初めにリミットスイッチで原点を取るが開始時に色々面倒だから
  }

  // Start ADC and save at DMA
//	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
//	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&arm_positions, 4);
  printf("start rest to home position\r\n");
  ResetToHomePosition();
	printf("Complete Initialize\r\n");


	// TODO: enable this func
	// Start timer interrupt (1kHz)
  // TODO delete
//	HAL_TIM_Base_Start_IT(&htim6);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Window = 99;
  hiwdg.Init.Reload = 99;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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
  hlpuart1.Init.BaudRate = 2000000;
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
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 80;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 9999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 80;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 9;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BoardLED_GPIO_Port, BoardLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Arm3Switch_Pin Arm2Switch_Pin Arm1Switch_Pin Arm0Switch_Pin */
  GPIO_InitStruct.Pin = Arm3Switch_Pin|Arm2Switch_Pin|Arm1Switch_Pin|Arm0Switch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BoardLED_Pin */
  GPIO_InitStruct.Pin = BoardLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BoardLED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
  HAL_GPIO_WritePin(BoardLED_GPIO_Port, BoardLED_Pin, GPIO_PIN_SET);
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
