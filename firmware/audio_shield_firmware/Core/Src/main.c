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
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "arm_const_structs.h"
#include "tapering_window.h"
#include "sweep_hard_bins.h"
#include "buzzer.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DCNotchActivated 1
//#define WINDOWINGActivated 1

#define N_ACTUAL_SAMPLES (2048)//32
#define HALF_BUFFER_SIZE (N_ACTUAL_SAMPLES * 2) // left + right microphones
#define FULL_BUFFER_SIZE (2 * HALF_BUFFER_SIZE)

// constants
#define N_MOTORS 4 // number of motors
#define N_MIC 4 // number of microphones
#define MAXINT 65535 // max for int16 (2**15)
#define FLOAT_PRECISION 4 // float = 4 bytes
#define INT16_PRECISION 2 // int16 = 2 bytes

#define FFTSIZE N_ACTUAL_SAMPLES // size of FFT (effectively we will have half samples because of symmetry)
#define FFTSIZE_SENT 32 // number of frequency bins to select

// processing
#define N_PROP_FACTORS 30
#define DF (32000.0/FFTSIZE)
#define ALPHA 0.5 // for IIR filtering

// cannot use both below flags at the same time
//#define USE_TEST_SIGNALS // set this to use test signals instead of real audio data. make sure SPI_N_BYTES MATCHES!

// communication
// in uint16, min_freq, max_freq, buzzer_freq_idx, n_average, delta_freq, snr_enable, propeller_enable, tot = 7
#define PARAMS_N_INT16 (N_MOTORS + 7)
#define CHECKSUM_VALUE (0xAC)

#define AUDIO_N_BYTES (N_MIC * FFTSIZE_SENT * FLOAT_PRECISION * 2)
#define FBINS_N_BYTES (FFTSIZE_SENT * INT16_PRECISION)
#define SPI_N_BYTES (AUDIO_N_BYTES + FBINS_N_BYTES + 4 + 1) // + 4 for timestamp, + 1 for checksum

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

// Timing tool using TIM2 in counter mode with us timebase.
#define STOPCHRONO ({\
		time_us = __HAL_TIM_GET_COUNTER(&htim2);\
		HAL_TIM_Base_Stop(&htim2);\
		HAL_TIM_Base_Init(&htim2);\
		HAL_TIM_Base_Start(&htim2);\
})
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2S_HandleTypeDef hi2s1;
I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi3_rx;

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */

// data vectors
int16_t dma_1[FULL_BUFFER_SIZE];
int16_t dma_3[FULL_BUFFER_SIZE];

#ifndef USE_TEST_SIGNALS
float mic0[N_ACTUAL_SAMPLES];
float mic2[N_ACTUAL_SAMPLES];
float mic1[N_ACTUAL_SAMPLES];
float mic3[N_ACTUAL_SAMPLES];
#else
//#include "real_data_1024.h"
#include "simulated_data_1024.h"
//#include "real_data_32.h"
#endif

float mic0_f[N_ACTUAL_SAMPLES]; // Complex type to feed rfft [real1,imag1, real2, imag2]
float mic2_f[N_ACTUAL_SAMPLES];
float mic1_f[N_ACTUAL_SAMPLES];
float mic3_f[N_ACTUAL_SAMPLES];

float mics_f_avg[4 * 2 * FFTSIZE_SENT]; // Complex type to feed rfft [real1, real2, imag1, imag2]
uint16_t f_avg_counter = 0;

#define NUM_TIM_FREQ 15

typedef const struct {
	uint16_t f;
	uint16_t PSC;
	uint16_t ARR;
	uint32_t ERR;
} freq_list_t;

#define NOTE_CHANGE_BY_TIMER

freq_list_t freq_list_tim[] = { { 3000, 26, 1038, 35 }, { 3125, 0, 26879, 0 }, {
		3250, 70, 363, 86 }, { 3375, 407, 60, 36 }, { 3500, 0, 23999, 0 }, {
		3625, 46, 492, 61 }, { 3750, 0, 22399, 0 }, { 3875, 52, 408, 19 }, {
		4000, 0, 20999, 0 }, { 4125, 10, 1850, 12 }, { 4250, 35, 548, 36 }, {
		4375, 0, 19199, 0 }, { 4500, 17, 1036, 36 }, { 4625, 2017, 8, 9 }, {
		4750, 1359, 12, 125 }, { 4875, 3445, 4, 45 } };

typedef enum {
	NOTE_WAIT_START, NOTE_WAIT, NOTE_RESET, NOTE_NEXT_NOTE
} state_note_t;

uint8_t flag_spi_recieved = 0;
uint8_t flag_reset_average = 0;
uint32_t note_tickstart;
uint16_t note_index = 0;

#define NOTE_SEQUENCE_LENGTH 16
#define NOTE_LENGTH 300

state_note_t state_note_sm = NOTE_WAIT_START;

uint8_t spi_tx_buffer[SPI_N_BYTES];
uint8_t spi_rx_buffer[SPI_N_BYTES];

float amplitude_avg[N_ACTUAL_SAMPLES / 2];
uint16_t selected_indices[FFTSIZE_SENT];

// parameters
uint16_t param_array[PARAMS_N_INT16];
uint16_t filter_prop_enable = 1;
uint16_t filter_snr_enable = 1;
uint16_t min_freq = 0;
uint16_t max_freq = 0;
uint16_t buzzer_freq_idx = 0;
uint16_t buzzer_freq_idx_old = 0;
uint16_t delta_freq = 100;
uint16_t n_average = 1; // number of frequency bins to average.
uint16_t n_added = 0; // counter of how many samples were averaged.

// audio processing
uint32_t timestamp;
uint32_t ifft_flag = 0;
uint8_t flag_fft_processing = 0;
uint8_t new_sample_to_process = 0;
uint8_t init_stage_iir = 1;
float prop_freq = 0;
arm_rfft_fast_instance_f32 rfft_instance;

// debugging
uint8_t retval = 0;
uint32_t counter_error = 0;
uint32_t counter_ok = 0;
volatile int32_t time_us;
volatile int32_t time_spi_error;
volatile int32_t time_spi_ok;

uint16_t freq = 0;

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
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
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
		process(dma_1, mic3, mic2, N_ACTUAL_SAMPLES);
	} else {
		process(dma_3, mic1, mic0, N_ACTUAL_SAMPLES);
	}
#endif
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s) {
#ifndef USE_TEST_SIGNALS
	if (hi2s->Instance == hi2s1.Instance) {
		process(&dma_1[HALF_BUFFER_SIZE], mic3, mic2, N_ACTUAL_SAMPLES);
	} else {
		process(&dma_3[HALF_BUFFER_SIZE], mic1, mic0, N_ACTUAL_SAMPLES);
	}
#endif
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
	STOPCHRONO;
	time_spi_error = time_us;
	counter_error += 1;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {

	HAL_GPIO_TogglePin(PC3_GPIO_Port, PC3_Pin);

	if ((spi_rx_buffer[SPI_N_BYTES - 1] != CHECKSUM_VALUE)) {
		STOPCHRONO;
		time_spi_error = time_us;
		counter_error += 1;
	} else {
		flag_spi_recieved = 1;
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
	MX_I2S3_Init();
	MX_TIM2_Init();
	MX_I2S1_Init();
	MX_SPI2_Init();
	MX_TIM5_Init();
	MX_TIM1_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */

	size_t length_tukey = sizeof(tukey_window) / sizeof(tukey_window[0]);
	assert(length_tukey == N_ACTUAL_SAMPLES);

	// Start DMAs
	HAL_I2S_Receive_DMA(&hi2s1, (uint16_t*) dma_1, FULL_BUFFER_SIZE);
	HAL_I2S_Receive_DMA(&hi2s3, (uint16_t*) dma_3, FULL_BUFFER_SIZE);

	memset(selected_indices, 0x00, sizeof(selected_indices));
	memset(amplitude_avg, 0x00, sizeof(amplitude_avg));

	HAL_TIM_Base_Init(&htim5);
	HAL_TIM_Base_Start(&htim5);
	timestamp = 0;

	// Super important! We need to wait until the bus is idle, otherwise
	// there is a random shift in the spi_rx_buffer and spi_tx_buffer.
	while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET) {
	};
	retval = HAL_SPI_TransmitReceive_DMA(&hspi2, spi_tx_buffer, spi_rx_buffer,
	SPI_N_BYTES);

	piezoInit();


	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	// Note that below while loop takes ca. 40ms (fft processing) + 10ms (bin selection and tx-buffer filling).
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

#ifdef USE_TEST_SIGNALS
		new_sample_to_process = 1;
#endif

		// Start and stop condition detection
		if (buzzer_freq_idx != buzzer_freq_idx_old) {
			if (buzzer_freq_idx == 0) {
				state_note_sm = NOTE_WAIT_START;
			} else {
				state_note_sm = NOTE_RESET;
			}
			buzzer_freq_idx_old = buzzer_freq_idx;
		}

		switch (state_note_sm) {
		case NOTE_RESET:
			note_index = 0;
			state_note_sm = NOTE_NEXT_NOTE;

			//memset(&mics_f_avg, 3, sizeof(mics_f_avg));

			for (size_t i = 0; i < 4*2*FFTSIZE_SENT; i++) {
				mics_f_avg[i] = 0;
			}

			f_avg_counter = 0;

			break;
		case NOTE_NEXT_NOTE:
			flag_reset_average = 1;
			HAL_TIM_Base_Init(&htim5);
			piezoSetMaxCount(freq_list_tim[note_index].ARR);
			piezoSetRatio(freq_list_tim[note_index].ARR / 2);
			piezoSetPSC(freq_list_tim[note_index].PSC);
			HAL_TIM_Base_Start(&htim5);

			note_tickstart = HAL_GetTick();
			state_note_sm = NOTE_WAIT;

			break;
		case NOTE_WAIT:
			// we have a new sample to process and want to add it to the buffer
			if (new_sample_to_process) {
				flag_fft_processing = 1;
				timestamp = __HAL_TIM_GET_COUNTER(&htim5);

				// Compute FFT
				arm_rfft_fast_init_f32(&rfft_instance, FFTSIZE);
				arm_rfft_fast_f32(&rfft_instance, mic0, mic0_f, ifft_flag);
				arm_rfft_fast_init_f32(&rfft_instance, FFTSIZE);
				arm_rfft_fast_f32(&rfft_instance, mic1, mic1_f, ifft_flag);
				arm_rfft_fast_init_f32(&rfft_instance, FFTSIZE);
				arm_rfft_fast_f32(&rfft_instance, mic2, mic2_f, ifft_flag);
				arm_rfft_fast_init_f32(&rfft_instance, FFTSIZE);
				arm_rfft_fast_f32(&rfft_instance, mic3, mic3_f, ifft_flag);

				// update the average amplitude for snr-based bin selection
				if (init_stage_iir) {
					for (int i = 0; i < N_ACTUAL_SAMPLES / 2; i++) {
						amplitude_avg[i] = abs_value_squared(&mic0_f[i * 2])
								+ abs_value_squared(&mic1_f[i * 2])
								+ abs_value_squared(&mic2_f[i * 2])
								+ abs_value_squared(&mic3_f[i * 2]);
					}
					init_stage_iir = 0;
				} else {
					for (int i = 0; i < N_ACTUAL_SAMPLES / 2; i++) {
						amplitude_avg[i] = (1 - ALPHA) * amplitude_avg[i]
								+ ALPHA
										* (abs_value_squared(&mic0_f[i * 2])
												+ abs_value_squared(
														&mic1_f[i * 2])
												+ abs_value_squared(
														&mic2_f[i * 2])
												+ abs_value_squared(
														&mic3_f[i * 2]));
					}
				}
				flag_fft_processing = 0;
				new_sample_to_process = 0;
			}
			frequency_bin_selection(selected_indices);
#if 1
			// Averaging between SPI communications
			uint8_t i_array = 0;
			for (int i_fbin = 0; i_fbin < FFTSIZE_SENT; i_fbin++) {
				mics_f_avg[i_array] += mic0_f[2 * selected_indices[i_fbin]]; i_array += 1;
				mics_f_avg[i_array] += mic1_f[2 * selected_indices[i_fbin]]; i_array += 1;
				mics_f_avg[i_array] += mic2_f[2 * selected_indices[i_fbin]]; i_array += 1;
				mics_f_avg[i_array] += mic3_f[2 * selected_indices[i_fbin]]; i_array += 1;
				mics_f_avg[i_array] += mic0_f[2 * selected_indices[i_fbin] + 1]; i_array += 1;
				mics_f_avg[i_array] += mic1_f[2 * selected_indices[i_fbin] + 1]; i_array += 1;
				mics_f_avg[i_array] += mic2_f[2 * selected_indices[i_fbin] + 1]; i_array += 1;
				mics_f_avg[i_array] += mic3_f[2 * selected_indices[i_fbin] + 1]; i_array += 1;
			}
			f_avg_counter++;
#else
			f_avg_counter = 1;
#endif
			fill_tx_buffer();
			read_rx_buffer();

#ifndef NOTE_CHANGE_BY_TIMER
			if((note_tickstart + NOTE_LENGTH) < HAL_GetTick()){
				note_index++;
				if(note_index == NOTE_SEQUENCE_LENGTH){
					state_note_sm = NOTE_WAIT_START;
				}else{
					state_note_sm = NOTE_NEXT_NOTE;
				}
			}
#else


			if (flag_spi_recieved) {
				flag_spi_recieved = 0;
				note_index++;
				if (note_index == NOTE_SEQUENCE_LENGTH) {
					state_note_sm = NOTE_WAIT_START;
				} else {
					state_note_sm = NOTE_NEXT_NOTE;
				}
			}
#endif
			break;
		case NOTE_WAIT_START:
			piezoSetMaxCount(0);
			piezoSetRatio(0);
			piezoSetPSC(0);

			break;
		default:
			break;
		}
		/*
		 piezoSetMaxCount(freq_list_tim[note_index].ARR);
		 piezoSetRatio(freq_list_tim[note_index].ARR / 2);
		 piezoSetPSC(freq_list_tim[note_index].PSC);
		 */

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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
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
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S_APB1
			| RCC_PERIPHCLK_I2S_APB2;
	PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
	PeriphClkInitStruct.PLLI2S.PLLI2SP = RCC_PLLI2SP_DIV2;
	PeriphClkInitStruct.PLLI2S.PLLI2SM = 16;
	PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
	PeriphClkInitStruct.PLLI2S.PLLI2SQ = 2;
	PeriphClkInitStruct.PLLI2SDivQ = 1;
	PeriphClkInitStruct.I2sApb2ClockSelection = RCC_I2SAPB2CLKSOURCE_PLLI2S;
	PeriphClkInitStruct.I2sApb1ClockSelection = RCC_I2SAPB1CLKSOURCE_PLLI2S;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2S1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2S1_Init(void) {

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
	if (HAL_I2S_Init(&hi2s1) != HAL_OK) {
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
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void) {

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
	if (HAL_SPI_Init(&hspi2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI2_Init 2 */

	/* USER CODE END SPI2_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

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
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void) {

	/* USER CODE BEGIN TIM5_Init 0 */

	/* USER CODE END TIM5_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM5_Init 1 */

	/* USER CODE END TIM5_Init 1 */
	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 1;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 100;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim5) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim5) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM5_Init 2 */

	/* USER CODE END TIM5_Init 2 */
	HAL_TIM_MspPostInit(&htim5);

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(PC3_GPIO_Port, PC3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : PC3_Pin */
	GPIO_InitStruct.Pin = PC3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(PC3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
#ifdef DCNotchActivated
static inline int16_t DCNotch(int16_t x, uint8_t filter_index) {
	static int16_t x_prev[4] = { 0, 0, 0, 0 };
	static int16_t y_prev[4] = { 0, 0, 0, 0 };
	y_prev[filter_index] = (((int32_t) y_prev[filter_index] * 0x00007999) >> 16)
			- x_prev[filter_index] + x;
	x_prev[filter_index] = x;
	return y_prev[filter_index];
}
#endif

void inline process(int16_t *pIn, float *pOut1, float *pOut2, uint16_t size) {

	// size is N_ACTUAL_SAMPLES.
	// pIn is of size 2 * N_ACTUAL_SAMPLES, because we have left and right mic.
	// each pOut is of size N_ACTUAL_SAMPLES.

	// Do not interrupt FFT processing.
	if (flag_fft_processing == 0) {

		for (uint16_t i = 0; i < size; i += 1) {
#ifdef DCNotchActivated
			if (pIn == dma_1) {
#ifdef WINDOWINGActivated
				*pOut1++ = (float) DCNotch(*pIn++, 1) / (float) MAXINT * tukey_window[i];
				*pOut2++ = (float) DCNotch(*pIn++, 2) / (float) MAXINT * tukey_window[i];
#else
				*pOut1++ = (float) DCNotch(*pIn++, 1) / (float) MAXINT;
				*pOut2++ = (float) DCNotch(*pIn++, 2) / (float) MAXINT;
#endif
			} else { // pIn == dma_3
#ifdef WINDOWINGActivated
				*pOut1++ = (float) DCNotch(*pIn++, 3) / (float) MAXINT * tukey_window[i];
				*pOut2++ = (float) DCNotch(*pIn++, 4) / (float) MAXINT * tukey_window[i];
#else
				*pOut1++ = (float) DCNotch(*pIn++, 3) / (float) MAXINT;
				*pOut2++ = (float) DCNotch(*pIn++, 4) / (float) MAXINT;
#endif
			};
#else // not DCNotchActivated
#ifdef WINDOWINGActivated
			*pOut1++ = (float) *pIn++ / (float) MAXINT * tukey_window[i];
			*pOut2++ = (float) *pIn++ / (float) MAXINT * tukey_window[i];
#else
			*pOut1++ = (float) *pIn++ / (float) MAXINT;
			*pOut2++ = (float) *pIn++ / (float) MAXINT;
#endif
#endif
		}
		new_sample_to_process = 1;
	}

#if 0
	/* process the data through the CFFT/CIFFT module */
	arm_cfft_f32(&arm_cfft_sR_f32_len1024, mic0, ifft_flag, do_bit_reverse);

	/* process the data through the Complex Magnitude Module for
	 calculating the magnitude at each bin */
	arm_cmplx_mag_f32(mic0, test_output, FFTSIZE);

	/* Calculates max_value and returns corresponding BIN value */
	arm_max_f32(test_output, FFTSIZE, &max_value, &test_index);
#endif
}

void frequency_bin_selection(uint16_t *selected_indices) {

	// This happens in the beginning only, afterwards it only happens if there
	// was faulty communication between the Crazyflie and Audio deck.
	if (min_freq >= max_freq) {
		return;
	}
	// This is the key for hard-coded frequency values (best suited bins for buzzer effect sweep_hard)
	else if ((min_freq == 1) & (max_freq == 2)) {
		memcpy(selected_indices, sweep_hard_bins, sizeof(sweep_hard_bins));
		return;
	}

	float const prop_factors[N_PROP_FACTORS] = { 0.5, 1, 1.5, 2, 3, 4, 5, 6, 7,
			8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24,
			25, 26, 27, 28 };

	// TODO(FD): for the moment we just use the average thrust here. We could
	// also loop through all propeller frequencies and not use any of them,
	// that would be more appropriate if the thrust values vary a lot between motors.
	if (filter_prop_enable) {
		float average_thrust = 0;
		int counter = 0; // used to average only non-zero motors (e.g. for 1-motor experiments)

		// make sure to reset, otherwise we keep filtering out propellers noise
		// even when they are not turning anymore.
		prop_freq = 0;
		for (int i = 0; i < N_MOTORS; i++) {
			if (param_array[i] > 0) {
				average_thrust += (float) param_array[i];
				counter++;
			}
		}
		// this was the old and potentially more average formula in general,
		// however it can be simplified for the range of thrust values of interest,
		// which is between 45000 and 55000.
		// prop_freq = 3.27258551 * sqrt(average_thrust) - 26.41814899;
		// f(45000) = 671, f(55000) = 750
		// prop_freq = (750 - 671) / (55000 - 45000) * (thrust - 45000) + 671
		// 0.0079 = (750 - 671) / (55000 - 45000), 315.5 = 671 - 0.0079 * 45000
		if (counter > 0) {
			average_thrust /= counter;
			prop_freq = 315.5 + 0.0079 * average_thrust;

		}
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
		if (filter_prop_enable) {
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

	// if we have less candidates than required, we simply send all candidates
	if (potential_count <= FFTSIZE_SENT) {
		if (!filter_snr_enable) {
			memcpy(selected_indices, potential_indices,
					potential_count * INT16_PRECISION);
		}
		// we still do the sorting for consistency (for instance, if down the line
		// we rely on the first index to be the strongest)
		else {
			struct index_amplitude sort_this[potential_count];
			for (int i = 0; i < potential_count; i++) {
				sort_this[i].amplitude = amplitude_avg[potential_indices[i]];
				sort_this[i].index = potential_indices[i];
			}
			qsort(sort_this, potential_count, sizeof(sort_this[0]),
					compare_amplitudes);
			for (int i = 0; i < potential_count; i++) {
				selected_indices[i] = sort_this[i].index;
			}
		}

		// fill the remaining indices with zeroes
		memset(&selected_indices[potential_count], 0x00,
		INT16_PRECISION * (FFTSIZE_SENT - potential_count));
		return;
	}

	if (filter_snr_enable == 0) {
		// sample uniformly among remaining candidates
		float decimation = (float) potential_count / FFTSIZE_SENT;
		for (int i = 0; i < FFTSIZE_SENT; i++) {
			selected_indices[i] =
					potential_indices[(int) round(i * decimation)];
		}
	}

	int start_idx = 0;

	// put the buzzer frequency in the first bin, if requested
	if (filter_snr_enable == 2) {
		selected_indices[0] = buzzer_freq_idx;
		start_idx = 1;
	}
	// put the sorted frequencies in the next bins
	if (filter_snr_enable >= 1) {
		struct index_amplitude sort_this[potential_count];

		for (int i = 0; i < potential_count; i++) {
			sort_this[i].amplitude = amplitude_avg[potential_indices[i]];
			sort_this[i].index = potential_indices[i];
		}

		qsort(sort_this, potential_count, sizeof(sort_this[0]),
				compare_amplitudes);
		for (int i = start_idx; i < FFTSIZE_SENT; i++) {
			selected_indices[i] = sort_this[i - start_idx].index;
		}
	}

}

uint16_t i_array;

void fill_tx_buffer() {
	// Fill with FFT content
	// The spi_tx_buffer will be filled like
	// [m1_real[0], m2_real[0], m3_real[0], m4_real[0], m1_imag[0], m2_imag[0], m3_imag[0], m4_imag[0],
	//  m1_real[1], m2_real[1], m3_real[1], m4_real[1], m1_imag[1], m2_imag[1], m3_imag[1], m4_imag[1],
	//  ...
	//  m1_real[N], m2_real[N], m3_real[N], m4_real[N], m1_imag[N], m2_imag[N], m3_imag[N], m4_imag[N]]
	//  where N is FFTSIZE_SENT-1

#if 1

	for (int i = 0; i < 4 * 2 * FFTSIZE_SENT; i++) {
		mics_f_avg[i] /= f_avg_counter;
	}

	memcpy(&spi_tx_buffer, &mics_f_avg, sizeof(mics_f_avg));

	i_array = sizeof(mics_f_avg);

#else

		float_to_byte_array(mic0_f[2 * selected_indices[i_fbin]],
				&spi_tx_buffer[i_array]);
		i_array += 4;
		float_to_byte_array(mic1_f[2 * selected_indices[i_fbin]],
				&spi_tx_buffer[i_array]);
		i_array += 4;
		float_to_byte_array(mic2_f[2 * selected_indices[i_fbin]],
				&spi_tx_buffer[i_array]);
		i_array += 4;
		float_to_byte_array(mic3_f[2 * selected_indices[i_fbin]],
				&spi_tx_buffer[i_array]);
		i_array += 4;
		float_to_byte_array(mic0_f[2 * selected_indices[i_fbin] + 1],
				&spi_tx_buffer[i_array]);
		i_array += 4;
		float_to_byte_array(mic1_f[2 * selected_indices[i_fbin] + 1],
				&spi_tx_buffer[i_array]);
		i_array += 4;
		float_to_byte_array(mic2_f[2 * selected_indices[i_fbin] + 1],
				&spi_tx_buffer[i_array]);
		i_array += 4;
		float_to_byte_array(mic3_f[2 * selected_indices[i_fbin] + 1],
				&spi_tx_buffer[i_array]);
		i_array += 4;
	}
#endif

	// Fill with bins indices
	memcpy(&spi_tx_buffer[i_array], selected_indices, sizeof(selected_indices));
	i_array += sizeof(selected_indices);

	// Fill with timestamp
	memcpy(&spi_tx_buffer[i_array], &timestamp, sizeof(timestamp));
	i_array += sizeof(timestamp);

	assert(i_array == SPI_N_BYTES - 1);
	spi_tx_buffer[SPI_N_BYTES - 1] = CHECKSUM_VALUE;
}

void read_rx_buffer() {
	memcpy(param_array, spi_rx_buffer, sizeof(param_array));

	// Sometimes, because of faulty communication, the packet is broken and we get
	// impossible values for the parameters. In that case they should not be updated.
	if (param_array[N_MOTORS] >= param_array[N_MOTORS + 1])
		return;

	min_freq = param_array[N_MOTORS];
	max_freq = param_array[N_MOTORS + 1];
	//buzzer_freq_idx = param_array[N_MOTORS + 2];
	delta_freq = param_array[N_MOTORS + 3];
	n_average = param_array[N_MOTORS + 4];
	filter_prop_enable = param_array[N_MOTORS + 5];
	filter_snr_enable = param_array[N_MOTORS + 6];
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
	memcpy(output, &temp, sizeof(temp));
}

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
