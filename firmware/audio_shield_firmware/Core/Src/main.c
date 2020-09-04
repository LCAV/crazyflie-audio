/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "math.h"
#include "arm_const_structs.h"
#include "tapering_window.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//#define USE_TEST_SIGNALS
#ifdef USE_TEST_SIGNALS
#include "fft_bin_data.h"
#endif

#include <stdio.h>
#include <stdlib.h>

// Timing tool using TIM2 in counter mode with uS timebase.
#define STOPCHRONO ({\
		time_us = __HAL_TIM_GET_COUNTER(&htim2);\
		HAL_TIM_Base_Stop(&htim2);\
		HAL_TIM_Base_Init(&htim2);\
		HAL_TIM_Base_Start(&htim2);\
	})
volatile int32_t time_us;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define N_ACTUAL_SAMPLES (1024)//32

#define HALF_BUFFER_SIZE (N_ACTUAL_SAMPLES * 2) // left + right
#define FULL_BUFFER_SIZE (2 * HALF_BUFFER_SIZE)

#define MAXINT 65535

int16_t dma_1[FULL_BUFFER_SIZE];
int16_t dma_3[FULL_BUFFER_SIZE];

float left_1[N_ACTUAL_SAMPLES]; // Complex type to feed fft [real1,imag1, real2, imag2]
float right_1[N_ACTUAL_SAMPLES];
float left_3[N_ACTUAL_SAMPLES];
float right_3[N_ACTUAL_SAMPLES];

float left_1_f[N_ACTUAL_SAMPLES]; // Complex type to feed fft [real1,imag1, real2, imag2]
float right_1_f[N_ACTUAL_SAMPLES];
float left_3_f[N_ACTUAL_SAMPLES];
float right_3_f[N_ACTUAL_SAMPLES];

#define FFTSIZE N_ACTUAL_SAMPLES
#define N_MIC 4
#define FFTSIZE_SENT 32
#define ARRAY_SIZE N_MIC*FFTSIZE_SENT*4*2

#define N_MOTORS 4
#define INT16_PRECISION 2 // int16 = 2 bytes
#define SIZE_OF_PARAM_I2C 3 // in uint16, min_freq = 1, max_freq = 1, snr + propeller enable = 1, tot = 3
#define I2C_RECEIVE_LENGTH_INT16 (N_MOTORS+SIZE_OF_PARAM_I2C)
#define I2C_RECEIVE_LENGTH_BYTE I2C_RECEIVE_LENGTH_INT16 * INT16_PRECISION

#define FBINS_ARRAY_SIZE_BYTES FFTSIZE_SENT*INT16_PRECISION

#define N_PROP_FACTORS 32
#define DF (32000.0/FFTSIZE)
#define DELTA_F_PROP 100

uint8_t I2C_param_array_byte[I2C_RECEIVE_LENGTH_BYTE];
uint16_t I2C_param_array[I2C_RECEIVE_LENGTH_INT16];
uint16_t motorPower_array[N_MOTORS];
uint8_t filter_propellers_enable = 0;
uint8_t filter_snr_enable = 0;
uint16_t min_freq = 100;
uint16_t max_freq = 10000;

uint32_t ifftFlag = 0;
uint32_t time_fft;
volatile uint32_t time_bin_process;

uint8_t processing = 0;
uint8_t new_sample_to_send = 0;

uint8_t I2C_send_array[ARRAY_SIZE + FBINS_ARRAY_SIZE_BYTES];
uint16_t selected_indices[FFTSIZE_SENT];

uint8_t srcRows;
uint8_t srcColumns;

arm_rfft_fast_instance_f32 S;

arm_status status;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2S_HandleTypeDef hi2s1;
I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi3_rx;

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

#define SPI_BUF_SIZE 1

uint8_t spiTxBuffer[SPI_BUF_SIZE];
uint8_t spiRxBuffer[SPI_BUF_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S3_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2S1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

void Process(int16_t *pIn, float *pOut1, float *pOut2, uint16_t size);
void corr_matrix_to_corr_array(uint8_t byte_matrix[FFTSIZE_SENT][N_MIC * 2][4],
		uint8_t byte_array[ARRAY_SIZE]);
void float_to_byte_array(float input, uint8_t output[]);
void float_matrix_to_byte_matrix(float float_matrix[FFTSIZE_SENT][N_MIC * 2],
		uint8_t byte_matrix[FFTSIZE_SENT][N_MIC * 2][4]);
void uint8_array_to_uint16(uint8_t input[], uint16_t *output);

void frequency_bin_selection(uint16_t *selected_indices);
void receive_I2C_param();
void fill_send_array();
void int16_array_to_byte_array(uint16_t int16_array[], uint8_t byte_array[]);
void int16_to_byte_array(uint16_t input, uint8_t output[]);
void send_I2C_array(void);
int compare_amplitudes(const void *a, const void *b);
float abs_value(float real_imag[]);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

struct index_amplitude {
	float amplitude;
	int index;
};

int compare_amplitudes(const void *a, const void *b) {
	struct index_amplitude *a1 = (struct index_amplitude*) a;
	struct index_amplitude *a2 = (struct index_amplitude*) b;
	if ((*a1).amplitude > (*a2).amplitude)
		return -1;
	else if ((*a1).amplitude < (*a2).amplitude)
		return 1;
	else
		return 0;
}

float abs_value(float real_imag[]) {
	return sqrt(real_imag[0] * real_imag[0] + real_imag[1] * real_imag[1]);
}

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
#ifndef USE_TEST_SIGNALS
	if (hi2s->Instance == hi2s1.Instance) {
		Process(dma_1, left_1, right_1, HALF_BUFFER_SIZE);
	} else {
		Process(dma_3, left_3, right_3, HALF_BUFFER_SIZE);
	}
#endif
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s) {
#ifndef USE_TEST_SIGNALS
	if (hi2s->Instance == hi2s1.Instance) {
		Process(&dma_1[HALF_BUFFER_SIZE], left_1, right_1, HALF_BUFFER_SIZE);
	} else {
		Process(&dma_3[HALF_BUFFER_SIZE], left_3, right_3, HALF_BUFFER_SIZE);
	}
#endif
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
	/* Prevent unused argument(s) compilation warning */
	UNUSED(hspi);

}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	/* Prevent unused argument(s) compilation warning */
	UNUSED(hspi);

}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
	/* Prevent unused argument(s) compilation warning */
	UNUSED(hspi);

}

void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef *hspi) {
	/* Prevent unused argument(s) compilation warning */
	UNUSED(hspi);

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
  MX_I2S3_Init();
  MX_TIM2_Init();
  MX_I2S1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

	// Start DMAs
	HAL_I2S_Receive_DMA(&hi2s1, (uint16_t*) dma_1, FULL_BUFFER_SIZE);
	HAL_I2S_Receive_DMA(&hi2s3, (uint16_t*) dma_3, FULL_BUFFER_SIZE);

	spiTxBuffer[0] = 0xEF;

	HAL_SPI_TransmitReceive_DMA(&hspi2, spiTxBuffer, spiRxBuffer, SPI_BUF_SIZE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

#ifdef USE_TEST_SIGNALS
		for (uint16_t i = 0; i < N_ACTUAL_SAMPLES; i++) {
			left_1[i] = mic0[i];
			left_3[i] = mic1[i];
			right_1[i] = mic2[i];
			right_3[i] = mic3[i];
		}
		new_sample_to_send = 1;
#endif

		spiTxBuffer[0] += 1;
		spiTxBuffer[0] %= 0xFF;

		if (new_sample_to_send) {
			processing = 1;
			/* Process the data through the CFFT/CIFFT module */

			// FFT executed in the main loop to spare time in the interrupt routine
			arm_rfft_fast_init_f32(&S, FFTSIZE);
			arm_rfft_fast_f32(&S, left_1, left_1_f, ifftFlag);
			arm_rfft_fast_init_f32(&S, FFTSIZE);
			arm_rfft_fast_f32(&S, left_3, left_3_f, ifftFlag);
			arm_rfft_fast_init_f32(&S, FFTSIZE);
			arm_rfft_fast_f32(&S, right_1, right_1_f, ifftFlag);
			arm_rfft_fast_init_f32(&S, FFTSIZE);
			arm_rfft_fast_f32(&S, right_3, right_3_f, ifftFlag);
			processing = 0;

			STOPCHRONO;

			frequency_bin_selection(selected_indices);
			send_I2C_array();

			new_sample_to_send = 0;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S_APB1|RCC_PERIPHCLK_I2S_APB2;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SP = RCC_PLLI2SP_DIV2;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 16;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  PeriphClkInitStruct.PLLI2S.PLLI2SQ = 2;
  PeriphClkInitStruct.PLLI2SDivQ = 1;
  PeriphClkInitStruct.I2sApb2ClockSelection = RCC_I2SAPB2CLKSOURCE_PLLI2S;
  PeriphClkInitStruct.I2sApb1ClockSelection = RCC_I2SAPB1CLKSOURCE_PLLI2S;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2S1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S1_Init(void)
{

  /* USER CODE BEGIN I2S1_Init 0 */

  /* USER CODE END I2S1_Init 0 */

  /* USER CODE BEGIN I2S1_Init 1 */

  /* USER CODE END I2S1_Init 1 */
  hi2s1.Instance = SPI1;
  hi2s1.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s1.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s1.Init.DataFormat = I2S_DATAFORMAT_16B_EXTENDED;
  hi2s1.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s1.Init.AudioFreq = I2S_AUDIOFREQ_32K;
  hi2s1.Init.CPOL = I2S_CPOL_LOW;
  hi2s1.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s1.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S1_Init 2 */

  /* USER CODE END I2S1_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B_EXTENDED;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_32K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

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
  hspi2.Init.Mode = SPI_MODE_SLAVE;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void inline Process(int16_t *pIn, float *pOut1, float *pOut2, uint16_t size) {

	// Do not interrupt FFT processing in the middle
	if (processing == 0) {
		for (uint16_t i = 0; i < size; i += 2) {
			// Copy memory into buffer and apply tukey window
			*pOut1++ = (float) *pIn++ * tukey_window[i] / MAXINT;
			*pOut2++ = (float) *pIn++ * tukey_window[i] / MAXINT;
		}
		new_sample_to_send = 1;
	}

#if 0
	STOPCHRONO;
	/* Process the data through the CFFT/CIFFT module */
	arm_cfft_f32(&arm_cfft_sR_f32_len1024, left_1, ifftFlag, doBitReverse);

	/* Process the data through the Complex Magnitude Module for
	 calculating the magnitude at each bin */
	arm_cmplx_mag_f32(left_1, testOutput, FFTSIZE);

	STOPCHRONO;
	time_arm_cmplx_mag_f32 = time_us;

	/* Calculates maxValue and returns corresponding BIN value */
	arm_max_f32(testOutput, FFTSIZE, &maxValue, &testIndex);
#endif
}

void frequency_bin_selection(uint16_t *selected_indices) {

	// TODO(FD) read this from propellers
	uint16_t thrust = 43000;
	float const prop_factors[N_PROP_FACTORS] = { 0.5, 1, 1.5, 2, 3, 4, 5, 6, 7,
			8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24,
			25, 26, 27, 28, 29, 30 };
	float prop_freq = 3.27258551 * sqrt(thrust) - 26.41814899;

	int min_freq_idx = (int) min_freq / DF;
	int max_freq_idx = (int) max_freq / DF;

	// Will only be partially filled. Could implement this with
	// a dynamic list if it is necessary.
	uint16_t potential_indices[FFTSIZE];
	uint16_t potential_count = 0;

	// Fill bin candidates with the available ones.
	for (uint16_t i = min_freq_idx; i < max_freq_idx; i++) {
		uint8_t use_this = 1;
		if (filter_propellers_enable) {
			// TODO(FD) loop through all motors here
			for (uint8_t j = 0; j < N_PROP_FACTORS; j++) {
				if (fabs(
						(i * DF - (prop_factors[j] * prop_freq))) < DELTA_F_PROP)
					use_this = 0;
				break;
			}
		}
		if (use_this) {
			potential_indices[potential_count] = i;
			potential_count++;
		}

	}

	// TODO(FD): check that we have enough potential candidates left:
	// potential_count should be bigger than FFTSIZE_SENT.
	// Otherwise come up with a good strategy.

	if (!filter_snr_enable) {
		// Samples uniformly among remaining candidates
		float decimation = (float) potential_count / FFTSIZE_SENT;
		for (int i = 0; i < FFTSIZE_SENT; i++) {
			selected_indices[i] = potential_indices[(int) (i * decimation)];
		}
	}

	else {
		struct index_amplitude sort_this[potential_count];

		for (int i = 0; i < potential_count; i++) {
			sort_this[i].amplitude = abs_value(
					&left_1_f[2 * potential_indices[i]])
					+ abs_value(&left_3_f[2 * potential_indices[i]])
					+ abs_value(&right_1_f[2 * potential_indices[i]])
					+ abs_value(&right_3_f[2 * potential_indices[i]]);
			sort_this[i].index = potential_indices[i];
		}

		qsort(sort_this, potential_count, sizeof(sort_this[0]),
				compare_amplitudes);
		for (int i = 0; i < FFTSIZE_SENT; i++) {
			selected_indices[i] = sort_this[i].index;
		}
	}
}

void send_I2C_array() {
	uint16_t i_array = 0;

	for (int i_fbin = 0; i_fbin < FFTSIZE_SENT; i_fbin++) {
		float_to_byte_array(left_1_f[2 * selected_indices[i_fbin]],
				&I2C_send_array[i_array]);
		i_array += 4;
		float_to_byte_array(left_3_f[2 * selected_indices[i_fbin]],
				&I2C_send_array[i_array]);
		i_array += 4;
		float_to_byte_array(right_1_f[2 * selected_indices[i_fbin]],
				&I2C_send_array[i_array]);
		i_array += 4;
		float_to_byte_array(right_3_f[2 * selected_indices[i_fbin]],
				&I2C_send_array[i_array]);
		i_array += 4;
		float_to_byte_array(left_1_f[2 * selected_indices[i_fbin] + 1],
				&I2C_send_array[i_array]);
		i_array += 4;
		float_to_byte_array(left_3_f[2 * selected_indices[i_fbin] + 1],
				&I2C_send_array[i_array]);
		i_array += 4;
		float_to_byte_array(right_1_f[2 * selected_indices[i_fbin] + 1],
				&I2C_send_array[i_array]);
		i_array += 4;
		float_to_byte_array(right_3_f[2 * selected_indices[i_fbin] + 1],
				&I2C_send_array[i_array]);
		i_array += 4;
	}

	for (int i = 0; i < FBINS_ARRAY_SIZE_BYTES; i++) {
		int16_to_byte_array(selected_indices[i], &I2C_send_array[i_array]);
		i_array += 2;
	}

	//HAL_I2C_Slave_Transmit_DMA(&hi2c1, I2C_send_array, ARRAY_SIZE + FBINS_ARRAY_SIZE_BYTES);
}

void float_to_byte_array(float input, uint8_t output[]) {
	uint32_t temp = *((uint32_t*) &input);
	for (int i = 0; i < 4; i++) {
		output[i] = temp & 0xFF;
		temp >>= 8;
	}
}

void int16_to_byte_array(uint16_t input, uint8_t output[]) {
	uint32_t temp = *((uint32_t*) &input);
	for (int i = 0; i < INT16_PRECISION; i++) {
		output[i] = temp & 0xFF;
		temp >>= 8;
	}
}

void uint8_array_to_uint16(uint8_t input[], uint16_t *output) {
	for (int i = 0; i < INT16_PRECISION; i++) {
		*((uint8_t*) (output) + i) = input[i];
	}
}

void receive_I2C_param() {
	//HAL_I2C_Slave_Receive_DMA(&hi2c1, I2C_param_array_byte, I2C_RECEIVE_LENGTH_BYTE);

	for (int i = 0; i < I2C_RECEIVE_LENGTH_INT16; i++) {
		uint8_array_to_uint16(&I2C_param_array_byte[i * INT16_PRECISION],
				&I2C_param_array[i]);
	}
	motorPower_array[0] = I2C_param_array[0];
	motorPower_array[1] = I2C_param_array[1];
	motorPower_array[2] = I2C_param_array[2];
	motorPower_array[3] = I2C_param_array[3];
	min_freq = I2C_param_array[4];
	max_freq = I2C_param_array[5];
	filter_propellers_enable = I2C_param_array_byte[12];
	filter_snr_enable = I2C_param_array_byte[13];
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
