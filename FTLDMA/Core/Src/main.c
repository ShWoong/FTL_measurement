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
#include <Max31855k.h>
#include <string.h>
#include <filters.h>
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
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
volatile uint8_t tim4Flag = 0;
volatile uint8_t spi2Flag = 0;
volatile uint8_t tempFlag = 1;
uint32_t count = 0;
double temperature = 0.0;
uint16_t length = 0; //Length of stretch sensor
uint8_t load_buf[20];
float load;

float alpha = 0.98;  //Coefficient of integral filter
float prev_output = 0;  //Previous value of integral filter
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
int GetMatchingLength(float filteredValue);
int _write(int file, char* p, int len){
	HAL_UART_Transmit(&huart2, p, len, 1);
	return len;
}
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_SPI2_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_UART_Receive_IT(&huart3, load_buf, 6);

  if(!begin()){
        printf("Could not initialize thermocouple\r\n");
        while (1) HAL_Delay(10);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (spi2Flag == 1){
		  temperature = readCelsius();
		  spi2Flag = 0;
		  tempFlag = 1;
	  }

	  if (tim4Flag == 1 && spi2Flag == 0){
		  tim4Flag = 0;
		  float filtered2 = IntegralFilter(count, &prev_output, alpha);
		  float filtered = BWLPF(filtered, 4);
		  length = GetMatchingLength(filtered2);

		  printf("%.2f", temperature);
		  printf(",");
		  printf("%.2f", load);
		  printf(",");
		  printf("%" PRIu16 "\r\n", length);
		  count = 0;
		  TIM2->CNT = 0;
		  TIM2->CCR1 = 0;
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);

		  if (tempFlag == 1){
			  tempFlag = 0;
			   spi2read32();
		  }

		  if (temperature >= 80){

		  }
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  sConfigIC.ICFilter = 15;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim4.Init.Prescaler = 4;
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
  HAL_GPIO_WritePin(GPIOC, SPI2_CS_Pin|CG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_CS_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI2_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CG_Pin */
  GPIO_InitStruct.Pin = CG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(CG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA8 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance==TIM4){
		tim4Flag = 1;
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
		count = TIM2->CCR1;
	}
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI2) {
        spi2Flag = 1;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {
		sscanf((char *)load_buf, "%f", &load);
		HAL_UART_Receive_IT(&huart3, load_buf, 6); // 6바이?�� ?��?��
	}
}

int GetMatchingLength(float filteredValue) {
    if (filteredValue <= 29700.67472) return 110;
    else if (filteredValue > 29700.67472 && filteredValue <= 29831.26761) return 111;
    else if (filteredValue > 29831.26761 && filteredValue <= 29961.86051) return 112;
    else if (filteredValue > 29961.86051 && filteredValue <= 30092.4534) return 113;
    else if (filteredValue > 30092.4534 && filteredValue <= 30223.0463) return 114;
    else if (filteredValue > 30223.0463 && filteredValue <= 30356.7076) return 115;
    else if (filteredValue > 30356.7076 && filteredValue <= 30493.4373) return 116;
    else if (filteredValue > 30493.4373 && filteredValue <= 30630.16701) return 117;
    else if (filteredValue > 30630.16701 && filteredValue <= 30766.89671) return 118;
    else if (filteredValue > 30766.89671 && filteredValue <= 30903.62641) return 119;
    else if (filteredValue > 30903.62641 && filteredValue <= 31041.67236) return 120;
    else if (filteredValue > 31041.67236 && filteredValue <= 31181.03454) return 121;
    else if (filteredValue > 31181.03454 && filteredValue <= 31320.39672) return 122;
    else if (filteredValue > 31320.39672 && filteredValue <= 31459.75891) return 123;
    else if (filteredValue > 31459.75891 && filteredValue <= 31599.12109) return 124;
    else if (filteredValue > 31599.12109 && filteredValue <= 31734.76618) return 125;
    else if (filteredValue > 31734.76618 && filteredValue <= 31866.69417) return 126;
    else if (filteredValue > 31866.69417 && filteredValue <= 31998.62217) return 127;
    else if (filteredValue > 31998.62217 && filteredValue <= 32130.55017) return 128;
    else if (filteredValue > 32130.55017 && filteredValue <= 32262.47816) return 129;
    else if (filteredValue > 32262.47816 && filteredValue <= 32396.46047) return 130;
    else if (filteredValue > 32396.46047 && filteredValue <= 32532.49709) return 131;
    else if (filteredValue > 32532.49709 && filteredValue <= 32668.53371) return 132;
    else if (filteredValue > 32668.53371 && filteredValue <= 32804.57033) return 133;
    else if (filteredValue > 32804.57033 && filteredValue <= 32940.60695) return 134;
    else if (filteredValue > 32940.60695 && filteredValue <= 33074.44884) return 135;
    else if (filteredValue > 33074.44884 && filteredValue <= 33206.09599) return 136;
    else if (filteredValue > 33206.09599 && filteredValue <= 33337.74315) return 137;
    else if (filteredValue > 33337.74315 && filteredValue <= 33469.3903) return 138;
    else if (filteredValue > 33469.3903 && filteredValue <= 33601.03746) return 139;
    else if (filteredValue > 33601.03746 && filteredValue <= 33728.28677) return 140;
    else if (filteredValue > 33728.28677 && filteredValue <= 33851.13824) return 141;
    else if (filteredValue > 33851.13824 && filteredValue <= 33973.9897) return 142;
    else if (filteredValue > 33973.9897 && filteredValue <= 34096.84117) return 143;
    else if (filteredValue > 34096.84117 && filteredValue <= 34219.69264) return 144;
    else if (filteredValue > 34219.69264 && filteredValue <= 34339.75432) return 145;
    else if (filteredValue > 34339.75432 && filteredValue <= 34457.0262) return 146;
    else if (filteredValue > 34457.0262 && filteredValue <= 34574.29809) return 147;
    else if (filteredValue > 34574.29809 && filteredValue <= 34691.56997) return 148;
    else if (filteredValue > 34691.56997 && filteredValue <= 34808.84185) return 149;
    else if (filteredValue > 34808.84185 && filteredValue <= 34931.2963) return 150;
    else if (filteredValue > 34931.2963 && filteredValue <= 35058.93329) return 151;
    else if (filteredValue > 35058.93329 && filteredValue <= 35186.57029) return 152;
    else if (filteredValue > 35186.57029 && filteredValue <= 35314.20729) return 153;
    else if (filteredValue > 35314.20729 && filteredValue <= 35441.84429) return 154;
    else if (filteredValue > 35441.84429 && filteredValue <= 35564.20613) return 155;
    else if (filteredValue > 35564.20613 && filteredValue <= 35681.29283) return 156;
    else if (filteredValue > 35681.29283 && filteredValue <= 35798.37953) return 157;
    else if (filteredValue > 35798.37953 && filteredValue <= 35915.46623) return 158;
    else if (filteredValue > 35915.46623 && filteredValue <= 36032.55292) return 159;
    else if (filteredValue > 36032.55292 && filteredValue <= 36152.67354) return 160;
    else if (filteredValue > 36152.67354 && filteredValue <= 36275.82808) return 161;
    else if (filteredValue > 36275.82808 && filteredValue <= 36398.98263) return 162;
    else if (filteredValue > 36398.98263 && filteredValue <= 36522.13717) return 163;
    else if (filteredValue > 36522.13717 && filteredValue <= 36645.29171) return 164;
    else if (filteredValue > 36645.29171 && filteredValue <= 36768.16159) return 165;
    else if (filteredValue > 36768.16159 && filteredValue <= 36890.74682) return 166;
    else if (filteredValue > 36890.74682 && filteredValue <= 37013.33204) return 167;
    else if (filteredValue > 37013.33204 && filteredValue <= 37135.91727) return 168;
    else if (filteredValue > 37135.91727 && filteredValue <= 37258.50249) return 169;
    else if (filteredValue > 37258.50249 && filteredValue <= 37378.18739) return 170;
    else if (filteredValue > 37378.18739 && filteredValue <= 37494.97195) return 171;
    else if (filteredValue > 37494.97195 && filteredValue <= 37611.75651) return 172;
    else if (filteredValue > 37611.75651 && filteredValue <= 37728.54107) return 173;
    else if (filteredValue > 37728.54107 && filteredValue <= 37845.32563) return 174;
    else if (filteredValue > 37845.32563 && filteredValue <= 37961.7628) return 175;
    else if (filteredValue > 37961.7628 && filteredValue <= 38077.85259) return 176;
    else if (filteredValue > 38077.85259 && filteredValue <= 38193.94238) return 177;
    else if (filteredValue > 38193.94238 && filteredValue <= 38310.03218) return 178;
    else if (filteredValue > 38310.03218 && filteredValue <= 38426.12197) return 179;
    else if (filteredValue > 38426.12197 && filteredValue <= 38541.81482) return 180;
    else if (filteredValue > 38541.81482 && filteredValue <= 38657.11075) return 181;
    else if (filteredValue > 38657.11075 && filteredValue <= 38772.40668) return 182;
    else if (filteredValue > 38772.40668 && filteredValue <= 38887.7026) return 183;
    else if (filteredValue > 38887.7026 && filteredValue <= 39002.99853) return 184;
    else if (filteredValue > 39002.99853 && filteredValue <= 39121.24609) return 185;
    else if (filteredValue > 39121.24609 && filteredValue <= 39242.4453) return 186;
    else if (filteredValue > 39242.4453 && filteredValue <= 39363.6445) return 187;
    else if (filteredValue > 39363.6445 && filteredValue <= 39484.8437) return 188;
    else if (filteredValue > 39484.8437 && filteredValue <= 39606.0429) return 189;
    else if (filteredValue > 39606.0429 && filteredValue <= 39726.05644) return 190;
    else if (filteredValue > 39726.05644 && filteredValue <= 39844.88432) return 191;
    else if (filteredValue > 39844.88432 && filteredValue <= 39963.71221) return 192;
    else if (filteredValue > 39963.71221 && filteredValue <= 40082.54009) return 193;
    else if (filteredValue > 40082.54009 && filteredValue <= 40201.36798) return 194;
    else if (filteredValue > 40201.36798 && filteredValue <= 40318.46586) return 195;
    else if (filteredValue > 40318.46586 && filteredValue <= 40433.83375) return 196;
    else if (filteredValue > 40433.83375 && filteredValue <= 40549.20163) return 197;
    else if (filteredValue > 40549.20163 && filteredValue <= 40664.56952) return 198;
    else if (filteredValue > 40664.56952 && filteredValue <= 40779.93741) return 199;
    else if (filteredValue > 40779.93741 && filteredValue <= 40895.36491) return 200;
    else if (filteredValue > 40895.36491 && filteredValue <= 41010.85204) return 201;
    else if (filteredValue > 41010.85204 && filteredValue <= 41126.33917) return 202;
    else if (filteredValue > 41126.33917 && filteredValue <= 41241.8263) return 203;
    else if (filteredValue > 41241.8263 && filteredValue <= 41357.31343) return 204;
    else if (filteredValue > 41357.31343 && filteredValue <= 41473.14217) return 205;
    else if (filteredValue > 41473.14217 && filteredValue <= 41589.31251) return 206;
    else if (filteredValue > 41589.31251 && filteredValue <= 41705.48286) return 207;
    else if (filteredValue > 41705.48286 && filteredValue <= 41821.65321) return 208;
    else if (filteredValue > 41821.65321 && filteredValue <= 41937.82355) return 209;
    else if (filteredValue > 41937.82355) return 210;
    return 0; // 매칭?���?????? ?��?��
}


void Disable_Interrupts(void)
{
    HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn);

    HAL_NVIC_DisableIRQ(TIM2_IRQn);
}

void Enable_Interrupts(void)
{
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

    HAL_NVIC_EnableIRQ(TIM2_IRQn);
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
