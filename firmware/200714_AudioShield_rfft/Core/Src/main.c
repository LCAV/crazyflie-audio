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
#include "arm_math.h"
#include "arm_const_structs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fft_bin_data.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define USE_TEST_SIGNALS 1

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
#define N_ACTUAL_SAMPLES 32

#define HALF_BUFFER_SIZE (N_ACTUAL_SAMPLES * 2) // left + right
#define FULL_BUFFER_SIZE (2 * HALF_BUFFER_SIZE)

#define MAXINT 65535

int16_t dma_2[FULL_BUFFER_SIZE];
int16_t dma_3[FULL_BUFFER_SIZE];

float left_2[N_ACTUAL_SAMPLES]; // Complex type to feed fft [real1,imag1, real2, imag2]
float right_2[N_ACTUAL_SAMPLES];
float left_3[N_ACTUAL_SAMPLES];
float right_3[N_ACTUAL_SAMPLES];

float left_2_f[N_ACTUAL_SAMPLES]; // Complex type to feed fft [real1,imag1, real2, imag2]
float right_2_f[N_ACTUAL_SAMPLES];
float left_3_f[N_ACTUAL_SAMPLES];
float right_3_f[N_ACTUAL_SAMPLES];

#define FFTSIZE 32
#define nMic 4

uint32_t ifftFlag = 0;
uint32_t doBitReverse = 1;

uint32_t time_fft;
volatile uint32_t time_bin_process;

uint8_t processing = 0;

arm_matrix_instance_f32 matXf;
arm_matrix_instance_f32 matXfh;
arm_matrix_instance_f32 matR[FFTSIZE];
arm_matrix_instance_f32 matRinv[FFTSIZE];

float vect_Xf[nMic * 2];
float vect_Xfh[nMic * 2];
float vect_R[FFTSIZE][nMic * nMic * 2];

uint8_t srcRows;
uint8_t srcColumns;

arm_rfft_fast_instance_f32 S;

arm_status status;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi3_rx;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2S2_Init(void);
static void MX_I2S3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

void Process(int16_t *pIn, float *pOut1, float *pOut2, uint16_t size);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
#if !USE_TEST_SIGNALS
	if (hi2s->Instance == hi2s2.Instance) {
		Process(dma_2, left_2, right_2, HALF_BUFFER_SIZE);
	} else {
		Process(dma_3, left_3, right_3, HALF_BUFFER_SIZE);
	}
#endif
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s) {
#if !USE_TEST_SIGNALS
	if (hi2s->Instance == hi2s2.Instance) {
		Process(&dma_2[HALF_BUFFER_SIZE], left_2, right_2, HALF_BUFFER_SIZE);
	} else {
		Process(&dma_3[HALF_BUFFER_SIZE], left_3, right_3, HALF_BUFFER_SIZE);
	}
#endif
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_I2S2_Init();
	MX_I2S3_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */

	HAL_GPIO_WritePin(VCC_1_GPIO_Port, VCC_1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(VCC_2_GPIO_Port, VCC_2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(VCC_3_GPIO_Port, VCC_3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(VCC_4_GPIO_Port, VCC_4_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GND_1_GPIO_Port, GND_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GND_2_GPIO_Port, GND_2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GND_3_GPIO_Port, GND_3_Pin, GPIO_PIN_RESET);

	// Start DMAs
	HAL_I2S_Receive_DMA(&hi2s2, (uint16_t*) dma_2, FULL_BUFFER_SIZE);
	HAL_I2S_Receive_DMA(&hi2s3, (uint16_t*) dma_3, FULL_BUFFER_SIZE);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

#if USE_TEST_SIGNALS
		for (uint16_t i = 0; i < N_ACTUAL_SAMPLES; i++) {
			left_2[i] = mic0[i];
			left_3[i] = mic1[i];
			right_2[i] = mic2[i];
			right_3[i] = mic3[i];
		}
#endif

		//HAL_Delay(100);
		processing = 1;
		STOPCHRONO;
		/* Process the data through the CFFT/CIFFT module */

		arm_rfft_fast_init_f32(&S, FFTSIZE);
		arm_rfft_fast_f32(&S, left_2, left_2_f, ifftFlag);
		arm_rfft_fast_init_f32(&S, FFTSIZE);
		arm_rfft_fast_f32(&S, left_3, left_3_f, ifftFlag);
		arm_rfft_fast_init_f32(&S, FFTSIZE);
		arm_rfft_fast_f32(&S, right_2, right_2_f, ifftFlag);
		arm_rfft_fast_init_f32(&S, FFTSIZE);
		arm_rfft_fast_f32(&S, right_3, right_3_f, ifftFlag);

		STOPCHRONO;

		time_fft = time_us;

		// Matrix initialisation

		srcRows = nMic * 2;
		srcColumns = 1;
		arm_mat_init_f32(&matXf, srcRows, srcColumns, vect_Xf);

		srcRows = 1;
		srcColumns = nMic * 2;
		arm_mat_init_f32(&matXfh, srcRows, srcColumns, vect_Xfh);

		uint16_t f = 0;

		// Frequency bin processing
		for (f = 0; f < FFTSIZE; f += 2) {
			vect_Xf[0] = left_2_f[f];
			vect_Xf[1] = left_2_f[f + 1];
			vect_Xf[2] = left_3_f[f];
			vect_Xf[3] = left_3_f[f + 1];
			vect_Xf[4] = right_2_f[f];
			vect_Xf[5] = right_2_f[f + 1];
			vect_Xf[6] = right_3_f[f];
			vect_Xf[7] = right_3_f[f + 1];

			arm_cmplx_conj_f32(vect_Xf, vect_Xfh, nMic);

			srcRows = nMic;
			srcColumns = nMic * 2;
			arm_mat_init_f32(&matR[f], srcRows, srcColumns, vect_R[f]);

			status = arm_mat_cmplx_mult_f32(&matXf, &matXfh, &matR[f]);

			arm_mat_inverse_f32(&matR[f], &matRinv[f]);
		}

		STOPCHRONO;
		time_bin_process = time_us;

#if 0
		/* Calculating the magnitude at each bin */
		arm_cmplx_mag_f32(left_2, testOutput, FFTSIZE);

		/* Calculates maxValue and returns corresponding BIN value */
		testOutput[0] = 0;
		arm_max_f32(testOutput, FFTSIZE, &maxValue, &testIndex);

		processing = 0;

#endif

#if 0
	 STOPCHRONO;
	 HAL_Delay(125);
#endif

	}

	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S_APB1;
	PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
	PeriphClkInitStruct.PLLI2S.PLLI2SP = RCC_PLLI2SP_DIV2;
	PeriphClkInitStruct.PLLI2S.PLLI2SM = 16;
	PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
	PeriphClkInitStruct.PLLI2S.PLLI2SQ = 2;
	PeriphClkInitStruct.PLLI2SDivQ = 1;
	PeriphClkInitStruct.I2sApb1ClockSelection = RCC_I2SAPB1CLKSOURCE_PLLI2S;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2S2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2S2_Init(void) {

	/* USER CODE BEGIN I2S2_Init 0 */

	/* USER CODE END I2S2_Init 0 */

	/* USER CODE BEGIN I2S2_Init 1 */

	/* USER CODE END I2S2_Init 1 */
	hi2s2.Instance = SPI2;
	hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
	hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
	hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B_EXTENDED;
	hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
	hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_32K;
	hi2s2.Init.CPOL = I2S_CPOL_LOW;
	hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
	hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
	if (HAL_I2S_Init(&hi2s2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2S2_Init 2 */

	/* USER CODE END I2S2_Init 2 */

}

/**
 * @brief I2S3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2S3_Init(void) {

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
	if (HAL_I2S_Init(&hi2s3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2S3_Init 2 */

	/* USER CODE END I2S3_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 84;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 0xFFFFFFFF;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/** 
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
	/* DMA1_Stream3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LD2_Pin | VCC_4_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, VCC_3_Pin | VCC_2_Pin | VCC_1_Pin | GND_1_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GND_2_Pin | GND_3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : VCC_3_Pin VCC_2_Pin VCC_1_Pin */
	GPIO_InitStruct.Pin = VCC_3_Pin | VCC_2_Pin | VCC_1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : GND_1_Pin */
	GPIO_InitStruct.Pin = GND_1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GND_1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : VCC_4_Pin */
	GPIO_InitStruct.Pin = VCC_4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(VCC_4_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : GND_2_Pin GND_3_Pin */
	GPIO_InitStruct.Pin = GND_2_Pin | GND_3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void inline Process(int16_t *pIn, float *pOut1, float *pOut2, uint16_t size) {

	if (processing == 0) {
		for (uint16_t i = 0; i < size; i += 2) {
			*pOut1++ = (float) *pIn++ / MAXINT;
			*pOut2++ = (float) *pIn++ / MAXINT;
		}
	}

#if 0
	STOPCHRONO;
	/* Process the data through the CFFT/CIFFT module */
	arm_cfft_f32(&arm_cfft_sR_f32_len1024, left_2, ifftFlag, doBitReverse);

	/* Process the data through the Complex Magnitude Module for
	 calculating the magnitude at each bin */
	arm_cmplx_mag_f32(left_2, testOutput, FFTSIZE);

	STOPCHRONO;
	time_arm_cmplx_mag_f32 = time_us;

	/* Calculates maxValue and returns corresponding BIN value */
	arm_max_f32(testOutput, FFTSIZE, &maxValue, &testIndex);
#endif
}

/*
 void inline Process_3(int16_t *pIn, uint16_t size) {
 for (uint16_t i = 0; i < size/2; i++){
 left_3[i] = *pIn++;
 right_3[i] = *pIn++;
 }
 }
 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
