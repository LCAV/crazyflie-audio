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
 *
 * Microphone designations:
 *
 * for location: front is where sign points, looking from above
 * main.c  |  ROS  | color  | location    | motor  |
 * =================================================
 * left_1  | mic0  | blue   | front right | m1     |
 * left_3  | mic1  | yellow | back right  | m2     |
 * right_1 | mic2  | green  | front left  | m4     |
 * right_3 | mic3  | red    | back left   | m3     |
 * =================================================
 *
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

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

// Timing tool using TIM2 in counter mode with uS timebase.
#define STOPCHRONO ({\
		time_us = __HAL_TIM_GET_COUNTER(&htim2);\
		HAL_TIM_Base_Stop(&htim2);\
		HAL_TIM_Base_Init(&htim2);\
		HAL_TIM_Base_Start(&htim2);\
})
volatile int32_t time_us;
volatile int32_t time_spi_error;
volatile int32_t time_spi_ok;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//#define DEBUG_SPI // set this define to use smaller, fixed buffers.
#define N_ACTUAL_SAMPLES (2048)//32
//#define N_ACTUAL_SAMPLES (4096)//32

#define HALF_BUFFER_SIZE (N_ACTUAL_SAMPLES * 2) // left + right microphones
#define FULL_BUFFER_SIZE (2 * HALF_BUFFER_SIZE)

#define MAXINT 65535

int16_t dma_1[FULL_BUFFER_SIZE];
int16_t dma_3[FULL_BUFFER_SIZE];

float left_1[N_ACTUAL_SAMPLES];
float right_1[N_ACTUAL_SAMPLES];
float left_3[N_ACTUAL_SAMPLES];
float right_3[N_ACTUAL_SAMPLES];

float left_1_f[N_ACTUAL_SAMPLES]; // Complex type to feed rfft [real1,imag1, real2, imag2]
float right_1_f[N_ACTUAL_SAMPLES];
float left_3_f[N_ACTUAL_SAMPLES];
float right_3_f[N_ACTUAL_SAMPLES];

float amplitude_avg[N_ACTUAL_SAMPLES / 2];

#define FFTSIZE N_ACTUAL_SAMPLES
#define N_MIC 4
#define FFTSIZE_SENT 32
#define FLOAT_PRECISION 4
#define AUDIO_N_BYTES (N_MIC * FFTSIZE_SENT * FLOAT_PRECISION * 2)

#define N_MOTORS 4
#define INT16_PRECISION 2 // int16 = 2 bytes
// in uint16, min_freq, max_freq, n_average delta_freq, snr_enable, propeller_enable, tot = 6
#define PARAMS_N_INT16 (N_MOTORS + 6)

#define FBINS_N_BYTES (FFTSIZE_SENT * INT16_PRECISION)

#define N_PROP_FACTORS 30
#define DF (32000.0/FFTSIZE)

#define IIR_FILTERING
#define ALPHA 0.9

uint16_t param_array[PARAMS_N_INT16];
uint16_t filter_props_enable = 1;
uint16_t filter_snr_enable = 1;
uint16_t min_freq = 100;
uint16_t max_freq = 10000;
uint16_t delta_freq = 100;
uint16_t n_average = 1; // number of frequency bins to average.
uint16_t n_added = 0; // counter of how many samples were averaged.

uint32_t ifft_flag = 0;
uint8_t flag_fft_processing = 0;
uint8_t new_sample_to_process = 0;
float prop_freq;
uint16_t selected_indices[FFTSIZE_SENT];

arm_rfft_fast_instance_f32 rfft_instance;

//#define USE_TEST_SIGNALS
#ifdef USE_TEST_SIGNALS
//#include "real_data_1024.h"
#include "simulated_data_1024.h"
//#include "real_data_32.h"
#endif

// This is somewhat related to the AUDIO_TASK_FREQUENCY on the Crazyflie. In the
// worse case we have to wait one full cycle before the Crazyflie starts transmitting,
// so this value should never be less than 1/AUDIO_TASK_FREQUENCY.
#define SPI_DEFAULT_TIMEOUT 100U

// DEBUGGING START
uint8_t retval = 0;
uint32_t waiting = 0;
uint32_t waiting_pin = 0;
uint32_t counter_error = 0;
uint32_t counter_ok = 0;
// DEBUGGING END

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
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */

#define CHECKSUM_VALUE 	0xAC
#define CHECKSUM_LENGTH 1
#define TIMESTAMP_LENGTH 4
uint8_t state = 0;

#ifdef DEBUG_SPI
#define SPI_N_BYTES 1100
#else
#define SPI_N_BYTES (AUDIO_N_BYTES + FBINS_N_BYTES + CHECKSUM_LENGTH + TIMESTAMP_LENGTH)
#endif

uint8_t spi_tx_buffer[SPI_N_BYTES];
uint8_t spi_rx_buffer[SPI_N_BYTES];

uint32_t timestamp;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S3_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2S1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

struct index_amplitude {
	float amplitude;
	int index;
};

void process(int16_t *pIn, float *pOut1, float *pOut2, uint16_t size);
void frequency_bin_selection(uint16_t *selected_indices);
void fill_tx_buffer();
void read_rx_buffer();
int compare_amplitudes(const void *a, const void *b);
float abs_value_squared(float real_imag[]);
void uint32_to_byte_array(uint32_t input, uint8_t output[]);
void float_to_byte_array(float input, uint8_t output[]);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
#ifndef USE_TEST_SIGNALS
	if (hi2s->Instance == hi2s1.Instance) {
		process(dma_1, left_1, right_1, HALF_BUFFER_SIZE);
	} else {
		process(dma_3, left_3, right_3, HALF_BUFFER_SIZE);
	}
#endif
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s) {
#ifndef USE_TEST_SIGNALS
	if (hi2s->Instance == hi2s1.Instance) {
		process(&dma_1[HALF_BUFFER_SIZE], left_1, right_1, HALF_BUFFER_SIZE);
	} else {
		process(&dma_3[HALF_BUFFER_SIZE], left_3, right_3, HALF_BUFFER_SIZE);
	}
#endif
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
	STOPCHRONO;
	time_spi_error = time_us;
	counter_error += 1;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	if ((spi_rx_buffer[SPI_N_BYTES - 1] != CHECKSUM_VALUE)) {
		STOPCHRONO;
		time_spi_error = time_us;
		counter_error += 1;
	}
	else {
		STOPCHRONO;
		time_spi_ok = time_us;
		counter_ok += 1;
	}
	// Note that we do not need to restart the transmission by caling
	// HAL_SPI_TransmitReceive_DMA again, because the buffer is circular.
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
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

	// Start DMAs
	HAL_I2S_Receive_DMA(&hi2s1, (uint16_t*) dma_1, FULL_BUFFER_SIZE);
	HAL_I2S_Receive_DMA(&hi2s3, (uint16_t*) dma_3, FULL_BUFFER_SIZE);

#ifdef DEBUG_SPI
	for (int j = 0; j < SPI_N_BYTES - 1; j++) {
		spi_tx_buffer[j] = j % 0xFF;
	}
	spi_tx_buffer[0] = 0xEF;
	spi_tx_buffer[SPI_N_BYTES - 1] = CHECKSUM_VALUE;
#endif
	//memset(spi_tx_buffer, 0x02, SPI_N_BYTES);
	memset(selected_indices, 0x00, FFTSIZE_SENT * 2);
	memset(amplitude_avg, 0x00, N_ACTUAL_SAMPLES * 2); // TODO (AH): *2 because of floats N_ACTUAL_SAMPLES/2 * 4 (float)

	HAL_TIM_Base_Init(&htim5);
	HAL_TIM_Base_Start(&htim5);
	timestamp = 0;

	// Super important! We need to wait until the bus is idle, otherwise
	// there is a random shift in the spi_rx_buffer and spi_tx_buffer.
	while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET) {};
	retval = HAL_SPI_TransmitReceive_DMA(&hspi2, spi_tx_buffer, spi_rx_buffer, SPI_N_BYTES);

	/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	// Note that below while loop takes ca. 40ms (fft processing) + 10ms (bin selection and tx-buffer filling).
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

#ifdef USE_TEST_SIGNALS
		memcpy(left_1, mic0, sizeof(mic0));
		memcpy(left_3, mic1, sizeof(mic1));
		memcpy(right_1, mic2, sizeof(mic2));
		memcpy(right_3, mic3, sizeof(mic3));
		new_sample_to_process = 1;
#endif

		// we have a new sample to process and want to add it to the buffer
		if (new_sample_to_process) {
			flag_fft_processing = 1;
			timestamp = __HAL_TIM_GET_COUNTER(&htim5);

			// Compute FFT
			arm_rfft_fast_init_f32(&rfft_instance, FFTSIZE);
			arm_rfft_fast_f32(&rfft_instance, left_1, left_1_f, ifft_flag);
			arm_rfft_fast_init_f32(&rfft_instance, FFTSIZE);
			arm_rfft_fast_f32(&rfft_instance, left_3, left_3_f, ifft_flag);
			arm_rfft_fast_init_f32(&rfft_instance, FFTSIZE);
			arm_rfft_fast_f32(&rfft_instance, right_1, right_1_f, ifft_flag);
			arm_rfft_fast_init_f32(&rfft_instance, FFTSIZE);
			arm_rfft_fast_f32(&rfft_instance, right_3, right_3_f, ifft_flag);

			// Sum into AVG buffer
#ifndef IIR_FILTERING
			for (int i = 0; i < N_ACTUAL_SAMPLES / 2; i++) {
				amplitude_avg[i] +=  (abs_value_squared(&left_1_f[i*2])
									+ abs_value_squared(&left_3_f[i*2])
									+ abs_value_squared(&right_1_f[i*2])
									+ abs_value_squared(&right_3_f[i*2])) / N_MIC;
			}
#else
			for (int i = 0; i < N_ACTUAL_SAMPLES / 2; i++) {
				amplitude_avg[i] = (1 - ALPHA) * amplitude_avg[i]
						 + ALPHA * (abs_value_squared(&left_1_f[i * 2])
									  	+ abs_value_squared(&left_3_f[i * 2])
									  	+ abs_value_squared(&right_1_f[i * 2])
									  	+ abs_value_squared(&right_3_f[i * 2]));
			}
#endif
			n_added += 1;
			flag_fft_processing = 0;
			new_sample_to_process = 0;
		}

		// We have reached the desired number of samples to average, so we fill the new tx_buffer
		// and then reset the average to zero.
		if (n_added == n_average) {
			frequency_bin_selection(selected_indices);

			// Reset Average buffer.
			n_added = 0;
			memset(amplitude_avg, 0x00, sizeof(amplitude_avg)); 
#ifndef DEBUG_SPI
			fill_tx_buffer();
			read_rx_buffer();
#endif
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
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
  /** Initializes the CPU, AHB and APB buses clocks
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
  hspi2.Init.NSS = SPI_NSS_HARD_INPUT;
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 84;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0xffffffff;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void inline process(int16_t *pIn, float *pOut1, float *pOut2, uint16_t size) {

	// Do not interrupt FFT processing in the middle.
	if (flag_fft_processing == 0) {
		for (uint16_t i = 0; i < size; i += 2) {
			// Copy memory into buffer and apply tukey window.
			//*pOut1++ = (float) *pIn++ * tukey_window[i] / (float) MAXINT;
			//*pOut2++ = (float) *pIn++ * tukey_window[i] / (float) MAXINT;
			//
			// TODO(FD): When we add * tukey_window[i], even if it is set to constant 1,
			// the order of magnitude of the FFT values changes. Need to figure out why.
			*pOut1++ = (float) *pIn++ / (float) MAXINT;
			*pOut2++ = (float) *pIn++ / (float) MAXINT;

		}
		new_sample_to_process = 1;
	}

#if 0
	/* process the data through the CFFT/CIFFT module */
	arm_cfft_f32(&arm_cfft_sR_f32_len1024, left_1, ifft_flag, do_bit_reverse);

	/* process the data through the Complex Magnitude Module for
	 calculating the magnitude at each bin */
	arm_cmplx_mag_f32(left_1, test_output, FFTSIZE);

	/* Calculates max_value and returns corresponding BIN value */
	arm_max_f32(test_output, FFTSIZE, &max_value, &test_index);
#endif
}

void frequency_bin_selection(uint16_t *selected_indices) {

	// This should never happen:
	if (min_freq >= max_freq) {
		return;
	}

	float const prop_factors[N_PROP_FACTORS] = { 0.5, 1, 1.5, 2, 3, 4, 5, 6, 7,
			8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24,
			25, 26, 27, 28};

	// TODO(FD): for the moment we just use the average thrust here. We could
	// also loop through all propeller frequencies and not use any of them,
	// that would be more appropriate if the thrust values vary a lot between motors.
	if (filter_props_enable) {
		float average_thrust = 0;
		for (int i = 0; i < N_MOTORS; i++) {
			average_thrust += (float) param_array[i] / N_MOTORS;
		}
		prop_freq = 3.27258551 * sqrt(average_thrust) - 26.41814899;
	}

	int min_freq_idx = (int) min_freq / DF;
	int max_freq_idx = (int) max_freq / DF;

	// Will only be partially filled. Could implement this with
	// a dynamic list if it is necessary in terms of memory.
	uint16_t potential_indices[FFTSIZE];
	uint16_t potential_count = 0;

	// Fill bin candidates with the available ones, after removing propeller frequencies.
	uint8_t min_fac = 0;
	for (uint16_t i = min_freq_idx; i < max_freq_idx; i++) {
		uint8_t use_this = 1;
		if (filter_props_enable) {
			for (uint8_t j = min_fac; j < N_PROP_FACTORS; j++) {
				if (fabs(((i * DF) - (prop_factors[j] * prop_freq)))
						< delta_freq) {
					use_this = 0;

					// For the next potential bins we only need
					// to check starting from here, because they are increasing.
					min_fac = j;
					break;
				}
			}
		}
		if (use_this) {
			potential_indices[potential_count] = i;
			potential_count++;
		}
	}

	// TODO(FD): Come up with a better strategy.
	if (potential_count < FFTSIZE_SENT) {
		return;
	}

	if (!filter_snr_enable) {
		// Samples uniformly among remaining candidates.
		float decimation = (float) potential_count / FFTSIZE_SENT;
		for (int i = 0; i < FFTSIZE_SENT; i++) {
			selected_indices[i] =
					potential_indices[(int) round(i * decimation)];
		}
	} else {
		struct index_amplitude sort_this[potential_count];

		for (int i = 0; i < potential_count; i++) {
			sort_this[i].amplitude = amplitude_avg[potential_indices[i]];
			sort_this[i].index = potential_indices[i];
		}

		qsort(sort_this, potential_count, sizeof(sort_this[0]),
				compare_amplitudes);
		for (int i = 0; i < FFTSIZE_SENT; i++) {
			selected_indices[i] = sort_this[i].index;
		}
	}
}

void fill_tx_buffer() {
	// Fill with FFT content
	// The spi_tx_buffer will be filled like
	// [m1_real[0], m2_real[0], m3_real[0], m4_real[0], m1_imag[0], m2_imag[0], m3_imag[0], m4_imag[0],
	//  m1_real[1], m2_real[1], m3_real[1], m4_real[1], m1_imag[1], m2_imag[1], m3_imag[1], m4_imag[1],
	//  ...
	//  m1_real[N], m2_real[N], m3_real[N], m4_real[N], m1_imag[N], m2_imag[N], m3_imag[N], m4_imag[N]]
    //  where N is FFTSIZE_SENT-1

	uint16_t i_array = 0;
	for (int i_fbin = 0; i_fbin < FFTSIZE_SENT; i_fbin++) {
		float_to_byte_array(left_1_f[2 * selected_indices[i_fbin]], &spi_tx_buffer[i_array]);
		i_array += 4;
		float_to_byte_array(left_3_f[2 * selected_indices[i_fbin]], &spi_tx_buffer[i_array]);
		i_array += 4;
		float_to_byte_array(right_1_f[2 * selected_indices[i_fbin]], &spi_tx_buffer[i_array]);
		i_array += 4;
		float_to_byte_array(right_3_f[2 * selected_indices[i_fbin]], &spi_tx_buffer[i_array]);
		i_array += 4;
		float_to_byte_array(left_1_f[2 * selected_indices[i_fbin] + 1], &spi_tx_buffer[i_array]);
		i_array += 4;
		float_to_byte_array(left_3_f[2 * selected_indices[i_fbin] + 1], &spi_tx_buffer[i_array]);
		i_array += 4;
		float_to_byte_array(right_1_f[2 * selected_indices[i_fbin] + 1], &spi_tx_buffer[i_array]);
		i_array += 4;
		float_to_byte_array(right_3_f[2 * selected_indices[i_fbin] + 1], &spi_tx_buffer[i_array]);
		i_array += 4;
	}

	// Fill with bins indices
	memcpy(&spi_tx_buffer[i_array], selected_indices, sizeof(selected_indices));
	i_array += sizeof(selected_indices);

	// Fill with timestamp
	uint32_to_byte_array(timestamp, &spi_tx_buffer[i_array]);
	i_array += sizeof(timestamp);

	assert(i_array == SPI_N_BYTES - 1);
	spi_tx_buffer[SPI_N_BYTES - 1] = CHECKSUM_VALUE;
}

void read_rx_buffer() {
	memcpy(param_array, spi_rx_buffer, PARAMS_N_INT16 * INT16_PRECISION);

	// Sometimes, because of faulty communication, the packet is broken and we get
	// impossible values for the parameters. In that case they should not be updated.
	if ((param_array[N_MOTORS] == 0) || (param_array[N_MOTORS] >= param_array[N_MOTORS + 1]))
		return;

	min_freq = param_array[N_MOTORS];
	max_freq = param_array[N_MOTORS + 1];
	delta_freq = param_array[N_MOTORS + 2];
	n_average = param_array[N_MOTORS + 3];
	filter_props_enable = param_array[N_MOTORS + 4];
	filter_snr_enable = param_array[N_MOTORS + 5];
}

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

// Wrong absolute value but should work faster without sqrt.
float abs_value_squared(float real_imag[]) {
	return (real_imag[0] * real_imag[0] + real_imag[1] * real_imag[1]);
}

void float_to_byte_array(float input, uint8_t output[]) {
	uint32_t temp = *((uint32_t*) &input);
	uint32_to_byte_array(temp, output);
}

void uint32_to_byte_array(uint32_t input, uint8_t output[]) {
	memcpy(output, &input, sizeof(input));
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
