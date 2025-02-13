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
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <inttypes.h>
#include <string.h>
#include <filters.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SBUF_SIZE 64
#define PI 3.14159265358979323846
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
/******FLAGS******/
volatile uint8_t tim4Flag = 0;
volatile uint8_t spi2Flag = 0;
volatile uint8_t tempFlag = 1;
volatile uint8_t expFlag = 0;
volatile uint8_t actFlag = 0;
volatile uint8_t adc1Flag = 0;

/******COUNT******/
uint32_t count = 0;
uint8_t exp_count = 0;
uint16_t sample_count = 0;
volatile uint32_t cycle_counter = 0;

/******SENSORS******/
float temp = 0.0;
float length = 0; //Length of stretch sensor
uint8_t force_buf[20];
float force;
uint8_t sign;
uint32_t adcval1[1];
double emg_raw = 0.0;
double EMGMAF = 0.0;

/******CONTROLLER******/
float pgain = 0.0;
float igain = 0.0;
float dgain = 0.0;

float target_temp = 30.0;
float target_force = 0.0;
float p_control = 0.0;
float i_control = 0.0;
float d_control = 0.0;
float pid_control = 0.0;
float real_error = 0.0;
float acc_error = 0.0;
float error_gap = 0.0;

uint8_t pid_buf[5];

volatile uint32_t lastTick = 0;
volatile float dt = 0.1f;
float prev_error = 0.0f;

/******FILTER******/
float alpha = 0.98;  //Coefficient of integral filter
float prev_output = 0;  //Previous value of integral filter

/******SERIAL COMMUNICATION SETTINGS******/
uint8_t uart4_rx_buffer[UART4_RX_BUFFER_SIZE];
uint8_t uart5_rx_buffer[UART5_RX_BUFFER_SIZE];

// 기타 UART/USART DMA 수신 버퍼
uint8_t usart1_rx_buffer[USART1_RX_BUFFER_SIZE];
uint8_t usart2_rx_buffer[USART2_RX_BUFFER_SIZE];
uint8_t usart3_rx_buffer[USART3_RX_BUFFER_SIZE];

// IMU 파싱 결과 저장 (각 3개: Roll, Pitch, Yaw)
float euler1[6] = {0};  // UART4
float euler2[6] = {0};  // UART5

float kneeAngleMAF = 0.0;

// IMU 데이터 파싱 완료 플래그
volatile uint8_t imu4_flag = 0;
volatile uint8_t imu5_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_UART5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM8_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */
float KneeAngleEstimation(void);
float GetMatchingLength(float filteredValue);
int _write(int file, char* p, int len){
	HAL_UART_Transmit(&huart2, (uint8_t*)p, len, 1);
	return len;
}

void UpdatePIDControl(float measured_temp);
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
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_TIM8_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_Base_Start(&htim8);
  HAL_ADC_Start_DMA(&hadc1, adcval1, 1);
  HAL_UART_Receive_DMA(&huart1, usart1_rx_buffer, USART1_RX_BUFFER_SIZE);
  HAL_UART_Receive_DMA(&huart2, usart2_rx_buffer, USART2_RX_BUFFER_SIZE);
  HAL_UART_Receive_DMA(&huart3, usart3_rx_buffer, USART3_RX_BUFFER_SIZE);
  HAL_UART_Receive_DMA(&huart4, uart4_rx_buffer, UART4_RX_BUFFER_SIZE);
  HAL_UART_Receive_DMA(&huart5, uart5_rx_buffer, UART5_RX_BUFFER_SIZE);
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
  __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
  __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
  /**
   * @EBIMU commands
   */

  /*HAL_UART_Transmit(&huart4, (uint8_t*)"<reset>", strlen("<reset>"), 100);
  HAL_UART_Transmit(&huart5, (uint8_t*)"<reset>", strlen("<reset>"), 100);
  HAL_Delay(500);*/

  HAL_UART_Transmit(&huart4, (uint8_t*)"<sb6>", strlen("<sb6>"), 100);
  HAL_UART_Transmit(&huart5, (uint8_t*)"<sb6>", strlen("<sb6>"), 100);
  HAL_Delay(500);

  HAL_UART_Transmit(&huart4, (uint8_t*)"<sor10>", strlen("<sor10>"), 100);
  HAL_UART_Transmit(&huart5, (uint8_t*)"<sor10>", strlen("<sor10>"), 100);
  HAL_Delay(500);

  HAL_UART_Transmit(&huart4, (uint8_t*)"<soc1>", strlen("<soc1>"), 100);
  HAL_UART_Transmit(&huart5, (uint8_t*)"<soc1>", strlen("<soc1>"), 100);
  HAL_Delay(500);

  HAL_UART_Transmit(&huart4, (uint8_t*)"<sem0>", strlen("<sem0>"), 100);
  HAL_UART_Transmit(&huart5, (uint8_t*)"<sem0>", strlen("<sem0>"), 100);
  HAL_Delay(500);

  // 오일러각 출력모드 설정
  HAL_UART_Transmit(&huart4, (uint8_t*)"<sof1>", strlen("<sof1>"), 100);
  HAL_UART_Transmit(&huart5, (uint8_t*)"<sof1>", strlen("<sof1>"), 100);
  HAL_Delay(500);

  HAL_UART_Transmit(&huart4, (uint8_t*)"<soa5>", strlen("<soa5>"), 100);
  HAL_UART_Transmit(&huart5, (uint8_t*)"<soa5>", strlen("<soa5>"), 100);
  HAL_Delay(500);

  // 현재 자세 기준 설정
  HAL_UART_Transmit(&huart4, (uint8_t*)"<cmo>", strlen("<cmo>"), 100);
  HAL_UART_Transmit(&huart5, (uint8_t*)"<cmo>", strlen("<cmo>"), 100);
  HAL_Delay(500);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
/******GET EMG DATA******/
	  if(adc1Flag == 1){
		  adc1Flag = 0;
		  emg_raw = adcval1[0];
		  double HPFEMG = EMGBWHPF(emg_raw);
		  double RECEMG = fabs(HPFEMG);
		  double LPFEMG = EMGBWLPF(RECEMG);
		  EMGMAF = MAFEMG(LPFEMG);
	  }

/******EXPERIMENT******/


/******MAIN******/
	  if (tim4Flag == 1){
		  tim4Flag = 0;

		  if (expFlag == 1) {
		  		  /*if (sample_count >= 500){
		  			  if (target_force < 15){
		  				  target_force++;
		  			  }
		  			  else if (target_force >= 15){
		  				  target_force--;
		  			  }
		  			  sample_count = 0;
		  		  }*/
		  		  float t = (cycle_counter % 1000) * 0.01f;
		  		          // 0.1Hz 사인 웨이브 계산 (주기 10초)
		  		          // -PI/2 위상을 적용하면 t = 0일 때 target_force = 0
		  		          target_force = ((sin(2 * PI * 0.1f * t - PI/2) + 1) / 2) * 15.0f;

		  		          cycle_counter++;  // 다음 사이클
		  	  }
		  //printf("1.12555\r\n");

		  //printf("%f\r\n", LPFEMG2);
		  //printf("%f\r\n", EMGMAF);
		  UpdateDeltaTime();
		  UpdatePIDControl(force);

		  float filtered_streth = BWLPF(count, 4);
		  length = GetMatchingLength(filtered_streth);
		  //float EMGHPF =
		  kneeAngleMAF = KneeAngleEstimation();  // 함수 반환값을 변수에 저장
		  //printf("%f\r\n", length);

		  //printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n", temp, length, force, kneeAngleMAF, euler1[0], euler2[0], target_force);
		  printf("%.2f,%.2f,%.2f,%.2f\r\n", temp, force, length, target_force);
		  //printf("%.2f, %.2f, %.2f\r\n", pgain, igain, dgain);
		  sample_count++;
		  count = 0;
		  TIM2->CNT = 0;
		  TIM2->CCR1 = 0;
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);

/******READ TEMPERATURE******/
		  /*if (tempFlag == 1){
			  tempFlag = 0;
			   spi2read32();
		  }*/
	  }
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
  sConfig.Channel = ADC_CHANNEL_1;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3999999999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 17999;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 14;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 59999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 44999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 230400;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 230400;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  huart2.Init.BaudRate = 460800;
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
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STERTH_CHARGE_GPIO_Port, STERTH_CHARGE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SMA_DIR_Pin|FAN_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : STERTH_CHARGE_Pin */
  GPIO_InitStruct.Pin = STERTH_CHARGE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(STERTH_CHARGE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SMA_DIR_Pin */
  GPIO_InitStruct.Pin = SMA_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(SMA_DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FAN_DIR_Pin */
  GPIO_InitStruct.Pin = FAN_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FAN_DIR_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* Timer interrupt callback */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance==TIM4){
		tim4Flag = 1;
	}
}

/* EXTI Callback */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == B1_Pin)  // B1 버튼에 연결된 핀인지 확인
    {
        // expFlag 토글: 0이면 활성화, 1이면 비활성화
        if(expFlag == 0)
        {
        	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
            expFlag = 1;
        }
        else
        {
        	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
            expFlag = 0;
            target_force = 0;
        }
    }
}

/**
 * @brief Captures the value of timer when the capacitor had charged for measuring time constant of the stretch sensor.
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
		count = TIM2->CCR1;
	}
}

/**
 * @brief Common UART Idle processing.
 *        Called from each UART IRQ Handler.
 */
void UART_IdleProcess(UART_HandleTypeDef *huart, uint8_t *rx_buffer, uint16_t buffer_size)
{
    if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(huart);
        HAL_UART_DMAStop(huart);
        uint16_t rx_length = buffer_size - __HAL_DMA_GET_COUNTER(huart->hdmarx);
        if(rx_length > 0)
        {
            ProcessData(rx_buffer, rx_length, huart);
        }
        HAL_UART_Receive_DMA(huart, rx_buffer, buffer_size);
    }
}

/**
 * @brief Process received data.
 *        For USART2, parse PID command; otherwise, echo data via huart2.
 */
void ProcessData(uint8_t* data, uint16_t len, UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
        char pid_cmd[128] = {0};
        uint16_t copy_len = (len < 127 ? len : 127);
        memcpy(pid_cmd, data, copy_len);
        pid_cmd[copy_len] = '\0';

        char cmd = pid_cmd[0];
        float value;
        if(sscanf(pid_cmd + 1, "%f", &value) == 1)
        {
            switch(cmd)
            {
                case 'P':
                    pgain = value;
                    //printf("Updated pgain: %.1f\r\n", pgain);
                    break;
                case 'I':
                    igain = value;
                    //printf("Updated igain: %.1f\r\n", igain);
                    break;
                case 'D':
                    dgain = value;
                    //printf("Updated dgain: %.1f\r\n", dgain);
                    break;
                default:
                    //printf("Unknown PID command: %c\r\n", cmd);
                    break;
            }
        }
        else
        {
            printf(pid_cmd);
        }
    }
    else if(huart->Instance == USART3){
		// uart3로 받은 데이터를 온도값으로 파싱하여 temp 변수에 저장
		char temp_str[128] = {0};
		uint16_t copy_len = (len < 127 ? len : 127);
		memcpy(temp_str, data, copy_len);
		temp_str[copy_len] = '\0';

		float new_temp;
		if(sscanf(temp_str, "%f", &new_temp) == 1)
		{
			temp = new_temp;
			//printf("Updated temp: %.2f\r\n", temp);
		}
		else
		{
			//printf("Failed to parse temp: %s\r\n", temp_str);
		}
	}
    else if(huart->Instance == USART1){
		// uart1로 받은 데이터를 온도값으로 파싱하여 force 변수에 저장
		char force_str[128] = {0};
		uint16_t copy_len = (len < 127 ? len : 127);
		memcpy(force_str, data, copy_len);
		force_str[copy_len] = '\0';

		float new_force;
		if(sscanf(force_str, "%f", &new_force) == 1)
		{
			force = new_force;
			//printf("Updated force: %.2f\r\n", force);
		}
		else
		{
			//printf("Failed to parse temp: %s\r\n", temp_str);
		}
	}
    else if (huart->Instance == UART4) {
        char buf[SBUF_SIZE] = {0};
        uint16_t copy_len = (len < (SBUF_SIZE - 1) ? len : (SBUF_SIZE - 1));
        memcpy(buf, data, copy_len);
        buf[copy_len] = '\0';

        // ✅ '*' 문자가 있는지 확인
        if (buf[0] != '*') {
            return;  // '*'이 없으면 데이터 무시
        }

        char *payload = buf + 1;  // '*' 다음부터 데이터 시작

        // ✅ CR LF 제거
        int payload_len = strlen(payload);
        if (payload_len > 1 && (payload[payload_len - 2] == '\r' && payload[payload_len - 1] == '\n')) {
            payload[payload_len - 2] = '\0';
        }

        // ✅ 데이터 파싱
        char *token = strtok(payload, ",");
        float items[6] = {0};
        int i = 0;

        while (token != NULL && i < 6) {
            if (isdigit(token[0]) || token[0] == '-' || token[0] == '.') {
                // 숫자일 때만 저장
                items[i] = atof(token);
                i++;
            }
            token = strtok(NULL, ",");
        }

        // ✅ 실수 데이터만 저장 (euler1)
        for (int j = 0; j < 6; j++) {
            euler1[j] = items[j];
        }

        imu4_flag = 1;
    }

    else if (huart->Instance == UART5) {
        char buf[SBUF_SIZE] = {0};
        uint16_t copy_len = (len < (SBUF_SIZE - 1) ? len : (SBUF_SIZE - 1));
        memcpy(buf, data, copy_len);
        buf[copy_len] = '\0';

        // ✅ '*' 문자가 있는지 확인
        if (buf[0] != '*') {
            return;
        }

        char *payload = buf + 1;

        // ✅ CR LF 제거
        int payload_len = strlen(payload);
        if (payload_len > 1 && (payload[payload_len - 2] == '\r' && payload[payload_len - 1] == '\n')) {
            payload[payload_len - 2] = '\0';
        }

        // ✅ 데이터 파싱
        char *token = strtok(payload, ",");
        float items[6] = {0};
        int i = 0;

        while (token != NULL && i < 6) {
            if (isdigit(token[0]) || token[0] == '-' || token[0] == '.') {
                // 숫자일 때만 저장
                items[i] = atof(token);
                i++;
            }
            token = strtok(NULL, ",");
        }

        // ✅ 실수 데이터만 저장 (euler2)
        for (int j = 0; j < 6; j++) {
            euler2[j] = items[j];
        }

        imu5_flag = 1;
    }



}
/**
 * @brief Estimating the Knee Angle.
 */
float KneeAngleEstimation(void) {
    static float lastKneeAngle = 0.0f;  // 이전 무릎 각도 저장 변수

    if (imu4_flag && imu5_flag) {
        float kneeAngle = euler2[0] - euler1[0];  // 두 IMU 센서의 Roll 값 차이
        if (kneeAngle > 180)
            kneeAngle -= 360;
        if (kneeAngle < -180)
            kneeAngle += 360;

        float filteredAngle = MAF(kneeAngle);  // MAF()는 이동평균 필터 함수 (별도로 구현)

        lastKneeAngle = filteredAngle;  // 새로운 값으로 업데이트
        imu4_flag = 0;
        imu5_flag = 0;
        return filteredAngle;
    }

    // 새로운 데이터가 준비되지 않았으면 이전 값을 반환
    return lastKneeAngle;
}


/**
 * @brief Returns matching length from filtered value of stretch sensor data.
 */
float GetMatchingLength(float filteredValue) {

	float result = 0.0083815 * filteredValue - 232.04;
	//return result;
	if(result<0){
		result = 0;
		return result;
	}
	else{
		return result;
	}
}

/**
 * @brief Update PID control based on measured force.
 */
void UpdatePIDControl(float measured_force)
{
    real_error = target_force - measured_force;
    acc_error += real_error * dt;
    error_gap = real_error - prev_error;
    prev_error = real_error;

    p_control = pgain * real_error;
    i_control = igain * acc_error;
    d_control = dgain * (error_gap / dt);

    pid_control = p_control + i_control + d_control;

    if(pid_control > 100.0f)
    {
        pid_control = 100.0f;
    }
    if(pid_control < -100.0f)
    {
        pid_control = -100.0f;
    }

    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim3);

    if(pid_control >= 0)
    {
        uint32_t heater_compare = (uint32_t)((pid_control / 100.0f) * period);
        if(temp >= 80)
        {
            heater_compare = 0;
        }
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, heater_compare);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
        float fan_duty = fabs(pid_control);
        uint32_t fan_compare = (uint32_t)((fan_duty / 100.0f) * period);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, fan_compare);
    }
}

/**
 * @brief Update timer (dt) using HAL_GetTick()
 */
void UpdateDeltaTime(void)
{
    uint32_t currentTick = HAL_GetTick();
    uint32_t diff;

    if(currentTick >= lastTick)
    {
        diff = currentTick - lastTick;
    }
    else
    {
        diff = (0xFFFFFFFF - lastTick) + currentTick + 1;
    }

    dt = diff / 1000.0f;
    if(dt < 0.0001f)
    {
        dt = 0.0001f;
    }
    lastTick = currentTick;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	if(hadc->Instance == ADC1){
		adc1Flag = 1;
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
  * @param  file: pointer to the source file0 name
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
