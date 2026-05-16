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
#include "adc.h"
#include "dma.h"
#include "app_fatfs.h"
#include "i2c.h"
#include "usart.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "Line_Follower.h"
#include "RingBuffer.h"
#include "SimpleParser.h"
#include "motor.h"
#include "LowPassFilter.h"
#include "TJ_MPU6050.h"
#include "map.h"
#include "odometry.h"
#include "robot_config.h"
#include "robot_settings.h"
#include <math.h>
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
	/*Defines*/
//	#define true 1;
//	#define false 0;

	//uint8_t DrivingOnMap = 1;
	uint8_t SDReadingReady = 0;
	#define ENDLINE '\n'

	/*Sensor*/
	#define SENSOR_SCALE 1 // sensor values multiplier -- for tests only

	/*MAP*/
	#define CURVATURE_COMPENSATION 0
	Map_t map;
	Odometry_t RobotOdom;
	/*FatFS variables*/
	FRESULT FatFsResult;
	FATFS SdFatFs;
	FIL SdCardFile;

	/*IMU*/
	RawData_Def myAccelRaw, myGyroRaw;
	ScaledData_Def myAccelScaled, myGyroScaled;
	MPU_ConfigTypeDef myMpuConfig;

	float Yaw;
	static float GyroZBiasDps = 0.0f;
	static float GyroZRawDps = 0.0f;
	static float GyroZCorrectedDps = 0.0f;
	/*Encoders*/
	#define PI_MOTOR_SPEED_REGULATION 1
	#define LOW_PASS_FILTER_ALPHA 0.7
	LineFollower_t GRUZIK;
	motor_t Motor_L;
	motor_t Motor_R;

	uint16_t timer_counter_L;
	uint16_t timer_counter_R;

	/*Rest of PIDR regulator*/
	uint64_t Sensors_read = 0x000000000000ULL;
	int Position;
	float Ki = 0;
	float Kr = 0;
	int P, I, D, R;
	int Last_error = 0;
	int Errors[10] = {0,0,0,0,0,0,0,0,0,0};
	int Error_sum = 0;
	int Last_end = 0;	// 0 -> Left, 1 -> Right
	int Last_idle = 0;
	float Speed_level = 1;
	int ARR = 4;
	int actives = 0;
	uint32_t LastEndTimer;

	/*Communication*/
    char buffer[28];
    uint8_t buffer_2[60];
    uint8_t RxData;
	RingBuffer_t RB, ReceiveBuffer;
    uint8_t ReceivedData[PARSER_LINE_BUFFER_SIZE];
	volatile uint8_t ReceivedLines;
	volatile uint8_t MappingAutoClosePending;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void PID_control();
int errors_sum (int index, int abs);
void past_errors (int error);
void forward_brake(int pos_right, int pos_left);
int QTR8_read();
void sharp_turn();
void motor_control (double pos_right, double pos_left);
static void Robot_StopOutputs(void);
static void Robot_ServiceTelemetry(void);
static void Robot_ServiceControl(void);
static void Robot_SendOdomTelemetry(void);
static void Robot_SendDebugTelemetry(void);
static void Robot_UpdateSpeedStats(void);
static float ClampMotorCommand(float speed);
static uint32_t LineSensorThresholdUs(void);
static void LineSensorDelayUs(uint32_t delay_us);
static void LineSensorSetInput(void);
static void LineSensorSetOutputHigh(void);
static uint8_t LineSensorIsActive(GPIO_PinState state);
void Robot_ResetOdometry(void);
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
  MX_LPUART1_UART_Init();
  MX_ADC1_Init();
  MX_I2C3_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  if (MX_FATFS_Init() != APP_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN 2 */
  	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&GRUZIK.Adc_Value, 1);

  	/*Initial values for driving on Optimized route*/
  	map.p = 90.0f;
  	map.i = 0;
  	map.d = 10.0f;

  	GRUZIK.state = PidFollowing;
  	GRUZIK.PowerMode = Stop;
  	GRUZIK.LineFollowing = 0u;


  	/*Set initial values for PID*/
    GRUZIK.Kp = 0.015;
  	GRUZIK.Kd = 0.085;
  	GRUZIK.Speed_offset = 0.014;
  	GRUZIK.treshold = ROBOT_LINE_SENSOR_DEFAULT_THRESHOLD_US;

  	GRUZIK.Base_speed_R = 80;
  	GRUZIK.Base_speed_L = 80;
  	GRUZIK.Max_speed_R = 80;
  	GRUZIK.Max_speed_L = 80;

  	/*Sharp turn speed*/
  	GRUZIK.Sharp_bend_speed_right= -50;
  	GRUZIK.Sharp_bend_speed_left= 100;
  	GRUZIK.Bend_speed_right= -50;
  	GRUZIK.Bend_speed_left= 100;

  	map.Kk = 0;
  	map.DefaultPlaybackSpeed = 80.0f;
  	Map_Reset(&map);
  	(void)RobotSettings_Load(&GRUZIK, &map);
  	Odometry_Init(&RobotOdom, ROBOT_ODOMETRY_GYRO_WEIGHT);
  	Robot_ResetOdometry();


    /*Start receiving data from Blue tooth*/
    HAL_UART_Receive_IT(&hlpuart1, &RxData, 1);

    /*encoders*/
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); // Left Encoder
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL); // Right Encoder

    //         Motor     KP    KI
    Motor_Init(&Motor_R, 0.1, 0.2);//0.1 0.2
    Motor_Init(&Motor_L, 0.1, 0.2);
    Motor_SetEncoderDirection(&Motor_L, ROBOT_LEFT_ENCODER_DIRECTION);
    Motor_SetEncoderDirection(&Motor_R, ROBOT_RIGHT_ENCODER_DIRECTION);
    LowPassFilter_Init(&Motor_R.EncoderRpmFilter, LOW_PASS_FILTER_ALPHA);
    LowPassFilter_Init(&Motor_L.EncoderRpmFilter, LOW_PASS_FILTER_ALPHA);
    LowPassFilter_Init(&Motor_L.MetersPerSecondLPF, LOW_PASS_FILTER_ALPHA);
    LowPassFilter_Init(&Motor_R.MetersPerSecondLPF, LOW_PASS_FILTER_ALPHA);

    /*SD Card mount check. Files are opened by parser commands when needed. */
    FatFsResult = f_mount(&SdFatFs, "", 1);
    SDReadingReady = (FatFsResult == FR_OK) ? 1u : 0u;
	/*Start timers and PWM on channels*/
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);//right pwm
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);//left pwm

	/*IMU initialization*/
	//1. Initialize the MPU6050 module and I2C
	MPU6050_Init(&hi2c3);

	//2. Configure parameters
	myMpuConfig.Accel_Full_Scale = AFS_SEL_16g;
	myMpuConfig.ClockSource = Internal_8MHz;
	myMpuConfig.CONFIG_DLPF = DLPF_184A_188G_Hz;
	myMpuConfig.Gyro_Full_Scale = FS_SEL_2000;
	myMpuConfig.Sleep_Mode_Bit = 0;  //1: sleep mode, 0: normal mode
	MPU6050_Config(&myMpuConfig);
	htim4.Instance->CNT = ROBOT_ENCODER_COUNTER_CENTER;
	htim1.Instance->CNT = ROBOT_ENCODER_COUNTER_CENTER;
	HAL_TIM_Base_Start_IT(&htim5);

    /*LED diodes initial set*/
    HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(ReceivedLines > 0u)
	  {
		  Parser_TakeLine(&ReceiveBuffer, ReceivedData);
		  Parser_Parse(ReceivedData,&GRUZIK);
		  ReceivedLines--;
	  }

	  if(MappingAutoClosePending != 0u)
	  {
		  MappingAutoClosePending = 0u;
		  Parser_FinishMappingAutoClosed(&GRUZIK);
	  }

	  Robot_ServiceControl();
	  Robot_ServiceTelemetry();
	  RobotSettings_Service(&GRUZIK, &map);


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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/*Interrupts*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == LPUART1)
	{
		if(RB_Write(&ReceiveBuffer, RxData) == RB_OK)
		{
			if(RxData == ENDLINE)
			{
				ReceivedLines++;
			}
		}
    	HAL_UART_Receive_IT(&hlpuart1,&RxData, 1);
	}
}
/*Encoders reading at 1KHz */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM5)
	{
		Motor_L.EncoderValue = __HAL_TIM_GET_COUNTER(&htim4);
		Motor_R.EncoderValue = __HAL_TIM_GET_COUNTER(&htim1);
		htim4.Instance->CNT = ROBOT_ENCODER_COUNTER_CENTER;
		htim1.Instance->CNT = ROBOT_ENCODER_COUNTER_CENTER;

		Motor_CalculateSpeed(&Motor_R);
		Motor_CalculateSpeed(&Motor_L);

		MPU6050_Get_Accel_Scale(&myAccelScaled);
		MPU6050_Get_Gyro_Scale(&myGyroScaled);

		GyroZRawDps = myGyroScaled.z * ROBOT_IMU_Z_DIRECTION;
		if((fabsf(GyroZRawDps) < ROBOT_GYRO_STATIONARY_DPS) &&
		   (fabsf(Motor_L.DistanceInMeasurement) < ROBOT_ENCODER_STATIONARY_M) &&
		   (fabsf(Motor_R.DistanceInMeasurement) < ROBOT_ENCODER_STATIONARY_M))
		{
			GyroZBiasDps += ROBOT_GYRO_BIAS_LEARN_RATE * (GyroZRawDps - GyroZBiasDps);
		}
		GyroZCorrectedDps = GyroZRawDps - GyroZBiasDps;

		Odometry_Update(&RobotOdom,
						Motor_L.DistanceInMeasurement,
						Motor_R.DistanceInMeasurement,
						GyroZCorrectedDps,
						ROBOT_CONTROL_DT_S);
		Map_SyncPose(&map, &RobotOdom.pose, RobotOdom.last_delta_distance_m);
		map.OriIMU = RobotOdom.last_delta_yaw_gyro_rad;
		GRUZIK.Yaw = RobotOdom.pose.yaw_rad;
		GRUZIK.Orientation = RobotOdom.pose.yaw_rad * RAD_TO_DEG;
	}
}
/*Functions*/
void Robot_ResetOdometry(void)
{
	Odometry_Reset(&RobotOdom);
	Map_SyncPose(&map, &RobotOdom.pose, 0.0f);
	GRUZIK.Yaw = 0.0f;
	GRUZIK.Orientation = 0.0f;
}

static void Robot_StopOutputs(void)
{
	motor_control(0.0, 0.0);
	HAL_GPIO_WritePin(Motor_STBY_GPIO_Port, Motor_STBY_Pin, GPIO_PIN_RESET);
}

static void Robot_UpdateSpeedStats(void)
{
	float current_speed = 0.5f * (Motor_R.MetersPerSecondLPF.output + Motor_L.MetersPerSecondLPF.output);
	if(current_speed < 0.0f)
	{
		current_speed = -current_speed;
	}

	GRUZIK.AverageSpeedSum += current_speed;
	GRUZIK.AverageSpeedNum += 1.0f;
	GRUZIK.AverageSpeed = GRUZIK.AverageSpeedSum / GRUZIK.AverageSpeedNum;

	if(current_speed > GRUZIK.MaxSpeed)
	{
		GRUZIK.MaxSpeed = current_speed;
	}
}

static void Robot_ServiceControl(void)
{
	static uint32_t last_control_ms = 0u;
	uint32_t now = HAL_GetTick();

	if((now - last_control_ms) < ROBOT_LINE_CONTROL_PERIOD_MS)
	{
		return;
	}
	last_control_ms = now;

	if(GRUZIK.PowerMode == Start)
	{
		if(GRUZIK.blockinterrups == 0u)
		{
			if((GRUZIK.state == UnMapping) && (SDReadingReady == 1u))
			{
				DriveOnMap(&map, &RobotOdom.pose);
			}
			else if((GRUZIK.state == UnSpeedMapping) && (SDReadingReady == 1u))
			{
				SpeedUnMapping(&map, &RobotOdom.pose);
			}
			else if((GRUZIK.state == Mapping) && (SDReadingReady == 1u))
			{
				float map_speed_mps = 0.5f * (Motor_R.MetersPerSecondLPF.output + Motor_L.MetersPerSecondLPF.output);
				MapUpdateV2(&map, &RobotOdom.pose, map_speed_mps, GRUZIK.Error_P, GRUZIK.Error_D);
				if(map.AutoClosed != 0u)
				{
					MappingAutoClosePending = 1u;
				}
			}
		}

		if(!((GRUZIK.state == UnMapping) && (map.PlaybackActive != 0u)) && (map.AutoClosed == 0u))
		{
			PID_control();
		}

		Robot_UpdateSpeedStats();

		if((GRUZIK.UnMappingDone == 1u) && (HAL_GetTick() >= (GRUZIK.DoneUnMappingTimer + 1750u)))
		{
			GRUZIK.PowerMode = Stop;
			Robot_StopOutputs();
		}
	}
	else
	{
		Parser_ServiceManualDriveTimeout(&GRUZIK);
		if((TireCleaningActive == 0u) && (ManualDriveActive == 0u))
		{
			Robot_StopOutputs();
		}
	}
}

static void Robot_ServiceTelemetry(void)
{
	static uint32_t last_telemetry_ms = 0u;

	if(TelemetryMode == TELEMETRY_OFF)
	{
		return;
	}

	uint32_t now = HAL_GetTick();
	if((now - last_telemetry_ms) < ROBOT_TELEMETRY_PERIOD_MS)
	{
		return;
	}
	last_telemetry_ms = now;

	if(TelemetryMode == TELEMETRY_DEBUG)
	{
		(void)QTR8_read();
		Robot_SendOdomTelemetry();
		Robot_SendDebugTelemetry();
	}
	else
	{
		Robot_SendOdomTelemetry();
	}
}

static void Robot_SendOdomTelemetry(void)
{
	char tx[192];
	int len = snprintf(tx, sizeof(tx),
					   "ODOM,%.4f,%.4f,%.2f,%.4f,%.3f,%.3f,%.2f,%.2f,%.2f\r\n",
					   RobotOdom.pose.x_m,
					   RobotOdom.pose.y_m,
					   RobotOdom.pose.yaw_rad * RAD_TO_DEG,
					   RobotOdom.total_distance_m,
					   Motor_L.MetersPerSecondLPF.output,
					   Motor_R.MetersPerSecondLPF.output,
					   GyroZCorrectedDps,
					   RobotOdom.encoder_yaw_rad * RAD_TO_DEG,
					   RobotOdom.gyro_yaw_rad * RAD_TO_DEG);

	if((len > 0) && (len < (int)sizeof(tx)))
	{
		HAL_UART_Transmit(&hlpuart1, (uint8_t *)tx, (uint16_t)len, 100);
	}
}

static void Robot_SendDebugTelemetry(void)
{
	char tx[256];
	int len = snprintf(tx, sizeof(tx),
					   "DBG,%d,%d,%d,%ld,%ld,%.1f,%.1f,%.4f,%.4f,%.2f,%.2f,%.2f,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
					   GRUZIK.SensorPosition,
					   GRUZIK.actives,
					   GRUZIK.Last_end,
					   (long)Motor_L.EncoderDeltaCounts,
					   (long)Motor_R.EncoderDeltaCounts,
					   Motor_L.EncoderRpmFilter.output,
					   Motor_R.EncoderRpmFilter.output,
					   Motor_L.DistanceTraveled,
					   Motor_R.DistanceTraveled,
					   GyroZRawDps,
					   GyroZBiasDps,
					   GyroZCorrectedDps,
					   GRUZIK.SensorValues[0],
					   GRUZIK.SensorValues[1],
					   GRUZIK.SensorValues[2],
					   GRUZIK.SensorValues[3],
					   GRUZIK.SensorValues[4],
					   GRUZIK.SensorValues[5],
					   GRUZIK.SensorValues[6],
					   GRUZIK.SensorValues[7],
					   GRUZIK.SensorValues[8],
					   GRUZIK.SensorValues[9],
					   GRUZIK.SensorValues[10],
					   GRUZIK.SensorValues[11]);

	if((len > 0) && (len < (int)sizeof(tx)))
	{
		HAL_UART_Transmit(&hlpuart1, (uint8_t *)tx, (uint16_t)len, 100);
	}
}

static float ClampMotorCommand(float speed)
{
	if(speed < 0.0f)
	{
		return 0.0f;
	}
	if(speed > 250.0f)
	{
		return 250.0f;
	}
	return speed;
}

static uint32_t LineSensorThresholdUs(void)
{
	uint32_t threshold_us = GRUZIK.treshold;
	if(threshold_us == 0u)
	{
		threshold_us = ROBOT_LINE_SENSOR_DEFAULT_THRESHOLD_US;
	}
	if(threshold_us < ROBOT_LINE_SENSOR_MIN_THRESHOLD_US)
	{
		threshold_us = ROBOT_LINE_SENSOR_MIN_THRESHOLD_US;
	}
	if(threshold_us > ROBOT_LINE_SENSOR_MAX_THRESHOLD_US)
	{
		threshold_us = ROBOT_LINE_SENSOR_MAX_THRESHOLD_US;
	}
	return threshold_us;
}

static void LineSensorDelayUs(uint32_t delay_us)
{
	uint16_t start = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
	while((uint16_t)((uint16_t)__HAL_TIM_GET_COUNTER(&htim3) - start) < delay_us)
	{
	}
}

static void LineSensorSetInput(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;

	GPIO_InitStruct.Pin = SENSOR10_Pin | SENSOR11_Pin | SENSOR12_Pin;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = SENSOR1_Pin | SENSOR2_Pin | SENSOR3_Pin | SENSOR4_Pin | SENSOR5_Pin;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = SENSOR7_Pin | SENSOR8_Pin | SENSOR9_Pin;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = SENSOR6_Pin;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

static void LineSensorSetOutputHigh(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

	GPIO_InitStruct.Pin = SENSOR10_Pin | SENSOR11_Pin | SENSOR12_Pin;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOA, SENSOR10_Pin | SENSOR11_Pin | SENSOR12_Pin, GPIO_PIN_SET);

	GPIO_InitStruct.Pin = SENSOR1_Pin | SENSOR2_Pin | SENSOR3_Pin | SENSOR4_Pin | SENSOR5_Pin;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOB, SENSOR1_Pin | SENSOR2_Pin | SENSOR3_Pin | SENSOR4_Pin | SENSOR5_Pin, GPIO_PIN_SET);

	GPIO_InitStruct.Pin = SENSOR7_Pin | SENSOR8_Pin | SENSOR9_Pin;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOC, SENSOR7_Pin | SENSOR8_Pin | SENSOR9_Pin, GPIO_PIN_SET);

	GPIO_InitStruct.Pin = SENSOR6_Pin;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOD, SENSOR6_Pin, GPIO_PIN_SET);
}

static uint8_t LineSensorIsActive(GPIO_PinState state)
{
#if ROBOT_LINE_SENSOR_ACTIVE_HIGH
	return (state == GPIO_PIN_SET) ? 1u : 0u;
#else
	return (state == GPIO_PIN_RESET) ? 1u : 0u;
#endif
}

void motor_control (double pos_right, double pos_left)

{
	#ifdef PI_MOTOR_SPEED_REGULATION
	{
		if (pos_left < 0 )
		{
			Motor_L.set_speed = (pos_left * -1);
			PI_Loop(&Motor_L);
			Motor_L.speed = ClampMotorCommand(Motor_L.speed);

			__HAL_TIM_SET_COMPARE (&htim2, TIM_CHANNEL_4, (uint32_t)((ARR*Motor_L.speed) * GRUZIK.Speed_level));//PWM_L
			HAL_GPIO_WritePin(Motor_L_A_GPIO_Port, Motor_L_B_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(Motor_L_B_GPIO_Port, Motor_L_A_Pin, GPIO_PIN_RESET);

		}
		else
		{
			Motor_L.set_speed = pos_left;
			PI_Loop(&Motor_L);
			Motor_L.speed = ClampMotorCommand(Motor_L.speed);

			__HAL_TIM_SET_COMPARE (&htim2, TIM_CHANNEL_4, (uint32_t)((ARR*Motor_L.speed) * GRUZIK.Speed_level));//PWM_L
			HAL_GPIO_WritePin(Motor_L_A_GPIO_Port, Motor_L_B_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Motor_L_B_GPIO_Port, Motor_L_A_Pin, GPIO_PIN_SET);
		}
		if (pos_right < 0 )
		{
			Motor_R.set_speed = (pos_right * -1);
			PI_Loop(&Motor_R);
			Motor_R.speed = ClampMotorCommand(Motor_R.speed);

			__HAL_TIM_SET_COMPARE (&htim2, TIM_CHANNEL_1, (uint32_t)((ARR*Motor_R.speed) * GRUZIK.Speed_level));//PWM_R
			HAL_GPIO_WritePin(Motor_R_A_GPIO_Port, Motor_R_A_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(Motor_R_B_GPIO_Port, Motor_R_B_Pin, GPIO_PIN_RESET);
		}
		else
		{
			Motor_R.set_speed = pos_right;
			PI_Loop(&Motor_R);
			Motor_R.speed = ClampMotorCommand(Motor_R.speed);

			__HAL_TIM_SET_COMPARE (&htim2, TIM_CHANNEL_1, (uint32_t)((ARR*Motor_R.speed) * GRUZIK.Speed_level));//PWM_R
			HAL_GPIO_WritePin(Motor_R_A_GPIO_Port, Motor_R_A_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Motor_R_B_GPIO_Port, Motor_R_B_Pin, GPIO_PIN_SET);
		}
	}
	#else
	{
		if (pos_left < 0 )
		{
			__HAL_TIM_SET_COMPARE (&htim2, TIM_CHANNEL_4, (uint32_t)((ARR*pos_left*-1) * GRUZIK.Speed_level));//PWM_L
			HAL_GPIO_WritePin(Motor_L_A_GPIO_Port, Motor_L_B_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(Motor_L_B_GPIO_Port, Motor_L_A_Pin, GPIO_PIN_RESET);

		}
		else
		{
			__HAL_TIM_SET_COMPARE (&htim2, TIM_CHANNEL_4, (uint32_t)((ARR*pos_left) * GRUZIK.Speed_level));//PWM_L
			HAL_GPIO_WritePin(Motor_L_A_GPIO_Port, Motor_L_B_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Motor_L_B_GPIO_Port, Motor_L_A_Pin, GPIO_PIN_SET);
		}
		if (pos_right < 0 )
		{
			__HAL_TIM_SET_COMPARE (&htim2, TIM_CHANNEL_1, (uint32_t)((ARR*pos_right* -1) * GRUZIK.Speed_level));//PWM_R
			HAL_GPIO_WritePin(Motor_R_A_GPIO_Port, Motor_R_A_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(Motor_R_B_GPIO_Port, Motor_R_B_Pin, GPIO_PIN_RESET);
		}
		else
		{
			__HAL_TIM_SET_COMPARE (&htim2, TIM_CHANNEL_1, (uint32_t)((ARR*pos_right) * GRUZIK.Speed_level));//PWM_R
			HAL_GPIO_WritePin(Motor_R_A_GPIO_Port, Motor_R_A_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Motor_R_B_GPIO_Port, Motor_R_B_Pin, GPIO_PIN_SET);
		}
	}
	#endif

}


void sharp_turn ()
{

	if (Last_idle < 25)
	{
		if (Last_end == 1)
			motor_control(GRUZIK.Sharp_bend_speed_right, GRUZIK.Sharp_bend_speed_left);
		else
			motor_control(GRUZIK.Sharp_bend_speed_left, GRUZIK.Sharp_bend_speed_right);
	}
	else
	{
		if (Last_end == 1)
			motor_control(GRUZIK.Bend_speed_right, GRUZIK.Bend_speed_left);
		else
			motor_control(GRUZIK.Bend_speed_left, GRUZIK.Bend_speed_right);
	}
}
int QTR8_read ()
{
	HAL_GPIO_WritePin(LEDON_GPIO_Port, LEDON_Pin, GPIO_PIN_SET);
	LineSensorSetOutputHigh();
	LineSensorDelayUs(ROBOT_LINE_SENSOR_CHARGE_US);
	LineSensorSetInput();
	LineSensorDelayUs(LineSensorThresholdUs());

	const GPIO_PinState sensor_state[12] = {
		HAL_GPIO_ReadPin(SENSOR1_GPIO_Port, SENSOR1_Pin),
		HAL_GPIO_ReadPin(SENSOR2_GPIO_Port, SENSOR2_Pin),
		HAL_GPIO_ReadPin(SENSOR3_GPIO_Port, SENSOR3_Pin),
		HAL_GPIO_ReadPin(SENSOR4_GPIO_Port, SENSOR4_Pin),
		HAL_GPIO_ReadPin(SENSOR5_GPIO_Port, SENSOR5_Pin),
		HAL_GPIO_ReadPin(SENSOR6_GPIO_Port, SENSOR6_Pin),
		HAL_GPIO_ReadPin(SENSOR7_GPIO_Port, SENSOR7_Pin),
		HAL_GPIO_ReadPin(SENSOR8_GPIO_Port, SENSOR8_Pin),
		HAL_GPIO_ReadPin(SENSOR9_GPIO_Port, SENSOR9_Pin),
		HAL_GPIO_ReadPin(SENSOR10_GPIO_Port, SENSOR10_Pin),
		HAL_GPIO_ReadPin(SENSOR11_GPIO_Port, SENSOR11_Pin),
		HAL_GPIO_ReadPin(SENSOR12_GPIO_Port, SENSOR12_Pin)
	};

	Sensors_read = 0x000000000000ULL;
	int pos = 0;
	int active = 0;

	for(uint8_t i = 0u; i < 12u; i++)
	{
		GRUZIK.SensorValues[i] = LineSensorIsActive(sensor_state[i]);
		if(GRUZIK.SensorValues[i] != 0u)
		{
			Sensors_read |= (1ULL << (i * 4u));
			pos += (int)(i + 1u) * 1000 * SENSOR_SCALE;
			active++;
		}
	}

	if((GRUZIK.SensorValues[0] != 0u) && (HAL_GetTick() > (LastEndTimer + 50u)))
	{
		LastEndTimer = HAL_GetTick();
		Last_end = 1;
	}
	if((GRUZIK.SensorValues[11] != 0u) && (HAL_GetTick() > (LastEndTimer + 50u)))
	{
		LastEndTimer = HAL_GetTick();
		Last_end = 0;
	}

	HAL_GPIO_WritePin(LEDON_GPIO_Port, LEDON_Pin, GPIO_PIN_RESET);

	actives = active;
	GRUZIK.actives = active;
	GRUZIK.Last_end = Last_end;

	if(active == 0)
	{
		Last_idle++;
		Position = (Last_end == 1) ? (1000 * SENSOR_SCALE) : (12000 * SENSOR_SCALE);
	}
	else
	{
		Last_idle = 0;
		Position = pos / active;
	}

	GRUZIK.Last_idle = Last_idle;
	GRUZIK.SensorPosition = Position;
	return Position;
}
void forward_brake(int pos_right, int pos_left)
{
	if (actives == 0)
		sharp_turn();
	else
	  motor_control(pos_right, pos_left);
}

void past_errors (int error)
{
  for (int i = 9; i > 0; i--)
      Errors[i] = Errors[i-1];
  Errors[0] = error;
}

int errors_sum (int index, int abs)
{
  int sum = 0;
  for (int i = 0; i < index; i++)
  {
    if (abs == 1 && Errors[i] < 0)
      sum += -Errors[i];
    else
      sum += Errors[i];
  }
  return sum;
}

void PID_control()
{
  uint16_t position = QTR8_read();
  int error = (6500 * SENSOR_SCALE) - position;
  past_errors(error);

  P = error;
  GRUZIK.P = P;
  GRUZIK.Error_P = P;
  I = errors_sum(5, 0);
  D = error - Last_error;
  GRUZIK.D = D;
  GRUZIK.Error_D = D;
  R = errors_sum(5, 1);
  Last_error = error;
  GRUZIK.Last_error = Last_error;

  int motorspeed = P*GRUZIK.Kp + I*Ki + D*GRUZIK.Kd;

  int motorspeedl = GRUZIK.Base_speed_L + motorspeed - R*Kr;
  int motorspeedr = GRUZIK.Base_speed_R - motorspeed - R*Kr;

  if (motorspeedl > GRUZIK.Max_speed_L)
    motorspeedl = GRUZIK.Max_speed_L;
  if (motorspeedr > GRUZIK.Max_speed_R)
    motorspeedr = GRUZIK.Max_speed_R;

	forward_brake(motorspeedr, motorspeedl);
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
