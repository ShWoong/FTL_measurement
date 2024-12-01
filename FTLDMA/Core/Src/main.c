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
#define ADC_BUFFER_SIZE 4
#define SAMPLE_COUNT 1000
#define PI 3.14159265358979
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t adcBuf[ADC_BUFFER_SIZE] = {0};
volatile uint8_t tim4Flag = 0;
volatile uint8_t spi2Flag = 0;
volatile uint8_t tempFlag = 1;
volatile uint8_t cgFlag = 0;
uint16_t adc_average = 0;
uint32_t count = 0;
float prevalue = 0.0;
float adc_value = 0.0;
double temperature = 0.0;
uint16_t length = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM4_Init(void);
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
  /* USER CODE BEGIN 2 */
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim4);
  if(!begin()){
        printf("Could not initialize thermocouple\r\n");
        while (1) HAL_Delay(10);
  }
  FIR_Init(151, 1, 500.0);
  float alpha = 0.98;  // ?��?�� 계수 (?��?��?���? 과거 ?��?��?�� �?중치 증�?)
  float prev_output = 0;  // 초기 출력�? (?��?�� ?��?�� 초기?��)

  printf("Tau\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (spi2Flag == 1){
		  temperature = readCelsius();
		  spi2Flag = 0;
		  tempFlag = 1;
		  //printf("%.2f\r\n", temperature);
	  }

	  if (tim4Flag == 1 && spi2Flag == 0){
		  tim4Flag = 0;
		  //printf("%" PRIu32 "\r\n", count);
		  //float filtered = MAF(count);
		  //float filtered_signal = FIR_Process(count);
		  float filtered2 = IntegralFilter(count, &prev_output, alpha);
		  float filtered = BWLPF(filtered2, 4);
		  length = GetMatchingLength(filtered);

		  //float filtered2 = BWLPF_1st(count, 3);
		  //printf("%.2f", temperature);
		  //printf("%" PRIu32, count);
		  //printf(",");
		  //printf("%.2f", filtered2);
		  //printf("%.2f", filtered);
		  //printf(",");
		  //printf("%.2f\r\n", filtered);
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
  htim2.Init.Prescaler = 69;
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
  huart2.Init.BaudRate = 921600;
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
/*void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
    	if (cgFlag == 1){
    		adc_value = HAL_ADC_GetValue(&hadc1);
    		adcFlag = 1;
    		cgFlag = 0;
    		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
    	}
    	else if(cgFlag == 0){
    		cgFlag = 1;
    		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
    	}
    }
}*/

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

int GetMatchingLength(float filteredValue) {
    if (filteredValue <= 413) return 105;
    else if (filteredValue > 413 && filteredValue <= 415) return 106;
    else if (filteredValue > 415 && filteredValue <= 417) return 107;
    else if (filteredValue > 417 && filteredValue <= 419) return 108;
    else if (filteredValue > 419 && filteredValue <= 421) return 109;
    else if (filteredValue > 421 && filteredValue <= 423) return 110;
    else if (filteredValue > 423 && filteredValue <= 425) return 111;
    else if (filteredValue > 425 && filteredValue <= 427) return 112;
    else if (filteredValue > 427 && filteredValue <= 429) return 113;
    else if (filteredValue > 429 && filteredValue <= 431) return 114;
    else if (filteredValue > 431 && filteredValue <= 433) return 115;
    else if (filteredValue > 433 && filteredValue <= 435) return 116;
    else if (filteredValue > 435 && filteredValue <= 437) return 117;
    else if (filteredValue > 437 && filteredValue <= 439) return 118;
    else if (filteredValue > 439 && filteredValue <= 441) return 119;
    else if (filteredValue > 441 && filteredValue <= 443) return 120;
    else if (filteredValue > 443 && filteredValue <= 445) return 121;
    else if (filteredValue > 445 && filteredValue <= 447) return 122;
    else if (filteredValue > 447 && filteredValue <= 449) return 123;
    else if (filteredValue > 449 && filteredValue <= 451) return 124;
    else if (filteredValue > 451 && filteredValue <= 453) return 125;
    else if (filteredValue > 453 && filteredValue <= 455) return 126;
    else if (filteredValue > 455 && filteredValue <= 457) return 127;
    else if (filteredValue > 457 && filteredValue <= 459) return 128;
    else if (filteredValue > 459 && filteredValue <= 461) return 129;
    else if (filteredValue > 461 && filteredValue <= 463) return 130;
    else if (filteredValue > 463 && filteredValue <= 465) return 131;
    else if (filteredValue > 465 && filteredValue <= 467) return 132;
    else if (filteredValue > 467 && filteredValue <= 469) return 133;
    else if (filteredValue > 469 && filteredValue <= 471) return 134;
    else if (filteredValue > 471 && filteredValue <= 473) return 135;
    else if (filteredValue > 473 && filteredValue <= 475) return 136;
    else if (filteredValue > 475 && filteredValue <= 477) return 137;
    else if (filteredValue > 477 && filteredValue <= 479) return 138;
    else if (filteredValue > 479 && filteredValue <= 481) return 139;
    else if (filteredValue > 481 && filteredValue <= 483) return 140;
    else if (filteredValue > 483 && filteredValue <= 485) return 141;
    else if (filteredValue > 485 && filteredValue <= 487) return 142;
    else if (filteredValue > 487 && filteredValue <= 489) return 143;
    else if (filteredValue > 489 && filteredValue <= 491) return 144;
    else if (filteredValue > 491 && filteredValue <= 493) return 145;
    else if (filteredValue > 493 && filteredValue <= 495) return 146;
    else if (filteredValue > 495 && filteredValue <= 497) return 147;
    else if (filteredValue > 497 && filteredValue <= 499) return 148;
    else if (filteredValue > 499 && filteredValue <= 501) return 149;
    else if (filteredValue > 501 && filteredValue <= 503) return 150;
    else if (filteredValue > 503 && filteredValue <= 505) return 151;
    else if (filteredValue > 505 && filteredValue <= 507) return 152;
    else if (filteredValue > 507 && filteredValue <= 509) return 153;
    else if (filteredValue > 509 && filteredValue <= 511) return 154;
    else if (filteredValue > 511 && filteredValue <= 513) return 155;
    else if (filteredValue > 513 && filteredValue <= 515) return 156;
    else if (filteredValue > 515 && filteredValue <= 517) return 157;
    else if (filteredValue > 517 && filteredValue <= 519) return 158;
    else if (filteredValue > 519 && filteredValue <= 521) return 159;
    else if (filteredValue > 521 && filteredValue <= 523) return 160;
    else if (filteredValue > 523 && filteredValue <= 525) return 161;
    else if (filteredValue > 525 && filteredValue <= 527) return 162;
    else if (filteredValue > 527 && filteredValue <= 529) return 163;
    else if (filteredValue > 529 && filteredValue <= 531) return 164;
    else if (filteredValue > 531 && filteredValue <= 533) return 165;
    else if (filteredValue > 533 && filteredValue <= 535) return 166;
    else if (filteredValue > 535 && filteredValue <= 537) return 167;
    else if (filteredValue > 537 && filteredValue <= 539) return 168;
    else if (filteredValue > 539 && filteredValue <= 541) return 169;
    else if (filteredValue > 541 && filteredValue <= 543) return 170;
    else if (filteredValue > 543 && filteredValue <= 545) return 171;
    else if (filteredValue > 545 && filteredValue <= 547) return 172;
    else if (filteredValue > 547 && filteredValue <= 549) return 173;
    else if (filteredValue > 549 && filteredValue <= 551) return 174;
    else if (filteredValue > 551 && filteredValue <= 553) return 175;
    else if (filteredValue > 553 && filteredValue <= 555) return 176;
    else if (filteredValue > 555 && filteredValue <= 557) return 177;
    else if (filteredValue > 557 && filteredValue <= 559) return 178;
    else if (filteredValue > 559 && filteredValue <= 561) return 179;
    else if (filteredValue > 561 && filteredValue <= 563) return 180;
    else if (filteredValue > 563 && filteredValue <= 565) return 181;
    else if (filteredValue > 565 && filteredValue <= 567) return 182;
    else if (filteredValue > 567 && filteredValue <= 569) return 183;
    else if (filteredValue > 569 && filteredValue <= 571) return 184;
    else if (filteredValue > 571 && filteredValue <= 573) return 185;
    else if (filteredValue > 573 && filteredValue <= 575) return 186;
    else if (filteredValue > 575 && filteredValue <= 577) return 187;
    else if (filteredValue > 577 && filteredValue <= 579) return 188;
    else if (filteredValue > 579 && filteredValue <= 581) return 189;
    else if (filteredValue > 581 && filteredValue <= 583) return 190;
    else if (filteredValue > 583 && filteredValue <= 585) return 191;
    else if (filteredValue > 585 && filteredValue <= 587) return 192;
    else if (filteredValue > 587 && filteredValue <= 589) return 193;
    else if (filteredValue > 589 && filteredValue <= 591) return 194;
    else if (filteredValue > 591 && filteredValue <= 593) return 195;
    else if (filteredValue > 593 && filteredValue <= 595) return 196;
    else if (filteredValue > 595 && filteredValue <= 597) return 197;
    else if (filteredValue > 597 && filteredValue <= 599) return 198;
    else if (filteredValue > 599 && filteredValue <= 601) return 199;
    else if (filteredValue > 601 && filteredValue <= 603) return 200;
    else if (filteredValue > 603 && filteredValue <= 605) return 201;
    else if (filteredValue > 605 && filteredValue <= 607) return 202;
    else if (filteredValue > 607 && filteredValue <= 609) return 203;
    else if (filteredValue > 609 && filteredValue <= 611) return 204;
    else if (filteredValue > 611 && filteredValue <= 613) return 205;

    return 0; // No match
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
