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

#include "hann_window.h"
#include "flattop_window.h"
#include "tukey_window.h"
#include "sound.h"
#include "led.h"
#include "piezo.h"

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

#define FFTSIZE 512 // number of complex-valued FFT coefficients
#define N_BUFFER (FFTSIZE * 2) // 2 for complex
#define TWO_N_BUFFER (N_BUFFER * 2) // 2 for left + right microphones

// constants
#define N_MOTORS 4 // number of motors
#define N_MICS 4 // number of microphones

//the "f" is super important! otherwise it is a double, and division by double takes too long
#define MAX_INT16 32767.0f // max for int16 (2**15-1).
#define MAX_UINT16 65535.0f // max for uint16 (2**15-1).

#define FLOAT_PRECISION 4 // float32_t = 4 bytes
#define INT16_PRECISION 2 // int16 = 2 bytes
#define N_COMPLEX 2 // [real imag]

#define FFTSIZE_SENT 32 // number of frequency bins to select
#define FFTSIZE_HALF 16 // for filter_snr = 3, number of bins to send before buzzer bin

// processing
#define N_PROP_FACTORS 30
#define FS 64000.0f
#define DF (FS/N_BUFFER)
#define WINDOW_FREQS 10 // number of frequencies to consider above and below current frequency, to find max
#define IIR_ALPHA 0.5 // set to 1 for no effect (equivalent to removing IIR_FILTERING flag)

// communication
// in uint16, min_freq, max_freq, buzzer_idx, n_average, delta_freq, snr_enable, propeller_enable, window, tot = 8
#define PARAMS_N_INT16 (N_MOTORS + 8)
#define CHECKSUM_VALUE (0xAC)

#define AUDIO_N_BYTES (N_MICS * FFTSIZE_SENT * FLOAT_PRECISION * N_COMPLEX)
#define FBINS_N_BYTES (FFTSIZE_SENT * INT16_PRECISION)
#define SPI_N_BYTES (AUDIO_N_BYTES + FBINS_N_BYTES + 4 + 1) // + 4 for timestamp, + 1 for checksum

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

// Timing tool using TIM2 in counter mode with us timebase.
// @formatter:off
#define STOPCHRONO ({\
		time_us = __HAL_TIM_GET_COUNTER(&htim2);\
		HAL_TIM_Base_Stop(&htim2);\
		HAL_TIM_Base_Init(&htim2);\
		HAL_TIM_Base_Start(&htim2);\
})


 #define min(a,b) ({\
	__typeof__ (a) _a = (a); \
	__typeof__ (b) _b = (b);\
   _a < _b ? _a : _b;\
})

 #define max(a,b) ({\
	__typeof__ (a) _a = (a); \
	__typeof__ (b) _b = (b);\
   _a > _b ? _a : _b;\
})

// @formatter:on
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
uint16_t dma_1[TWO_N_BUFFER];
uint16_t dma_3[TWO_N_BUFFER];

float32_t mk2[N_BUFFER];
float32_t mk1[N_BUFFER];
float32_t mk4[N_BUFFER];
float32_t mk3[N_BUFFER];

#include "hann_window.h"
#include "tukey_window.h"
#include "flattop_window.h"

float32_t mk2_f[N_BUFFER]; // Complex type to feed rfft [real1, imag1, real2, imag2, ...]
float32_t mk4_f[N_BUFFER];
float32_t mk1_f[N_BUFFER];
float32_t mk3_f[N_BUFFER];

float32_t mics_f_all[N_MICS * N_COMPLEX * FFTSIZE_SENT];

float32_t m1;
float32_t m2;
float32_t m3;
float32_t m4;

//#define BUZZER_CHANGE_BY_TIMER

uint16_t current_frequency = 0;
uint8_t wait_to_send = 1;
uint32_t note_tickstart;
int8_t melody_index = 0;
uint8_t note_index = 0;
uint16_t buffer_index;
uint16_t i_array;

state_note_t state_note_sm = BUZZER_IDLE;

uint8_t spi_tx_buffer[SPI_N_BYTES];
uint8_t spi_rx_buffer[SPI_N_BYTES];

float32_t magnitude_mk2[FFTSIZE];
uint16_t selected_indices[FFTSIZE_SENT];

// parameters
uint16_t param_array[PARAMS_N_INT16];
uint16_t filter_prop_enable = 1;
uint16_t bin_selection = 3;
uint16_t window_type = 0; // windowing scheme, 0: none, 1: hann, 2: flattop, 3: tukey(0.2)
uint16_t min_freq = 0;
uint16_t max_freq = 0;
uint16_t buzzer_idx = 0;
uint16_t delta_freq = 100;
uint16_t n_average = 1; // number of frequency bins to average.

// audio processing
uint32_t timestamp;
uint32_t ifft_flag = 0; // do rFFT not irFFT
uint8_t flag_fft_processing = 0;
uint8_t new_sample_to_process = 0;
uint8_t init_stage_iir = 1;
uint16_t n_added = 0; // counter of how many samples were averaged.
float32_t prop_freq = 0;
arm_rfft_fast_instance_f32 rfft_instance;
int16_t *tapering_window;
uint16_t buzzer_idx_old = 0;
uint16_t buzzer_wait_counter;

// debugging
uint32_t last_update_spi = 0;
uint8_t retval = 0;
uint32_t counter_error = 0;
uint32_t counter_ok = 0;
volatile int32_t time_us;
//volatile int32_t time_spi_error;
//volatile int32_t time_spi_ok;

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
	float32_t amplitude;
	int index;
};

void process(uint16_t *pIn, float32_t *pOut1, float32_t *pOut2);
void frequency_bin_selection(uint16_t *selected_indices);
uint8_t fill_tx_buffer();
void read_rx_buffer();
int compare_amplitudes(const void *a, const void *b);
float32_t abs_value_squared(float32_t real_imag[]);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
	if (hi2s->Instance == hi2s1.Instance) {
		process(dma_1, mic3, mic2);
	} else {
		process(dma_3, mic1, mic0);
	}
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s) {
	if (hi2s->Instance == hi2s1.Instance) {
		process(&dma_1[N_BUFFER], mic3, mic2);
	} else {
		process(&dma_3[N_BUFFER], mic1, mic0);
	}
}
*/
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s) {

	if (hi2s->Instance == hi2s1.Instance) {
		process(dma_1, mk3, mk4);
	} else {
		process(dma_3, mk1, mk2);
	}
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	if ((spi_rx_buffer[SPI_N_BYTES - 1] != CHECKSUM_VALUE)) {
		//STOPCHRONO;
		//time_spi_error = time_us;
		counter_error += 1;
	} else {
		last_update_spi = HAL_GetTick();

		// Set the CHECKSUM to 0 so that if we communicate during filling,
		// the package is not valid
		spi_tx_buffer[SPI_N_BYTES - 1] = 0;

		wait_to_send = 0;

		//STOPCHRONO;
		//time_spi_ok = time_us;
		counter_ok += 1;
	}
	// Note that we do not need to restart the transmission by calling
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
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

	// Start audio capture
	HAL_I2S_Receive_DMA(&hi2s1, (uint16_t*) dma_1, TWO_N_BUFFER);
	HAL_I2S_Receive_DMA(&hi2s3, (uint16_t*) dma_3, TWO_N_BUFFER);

	// Reset memory
	memset(selected_indices, 0x00, sizeof(selected_indices));
	memset(magnitude_mk2, 0x00, sizeof(magnitude_mk2));
	memset(spi_tx_buffer, 0x00, sizeof(spi_tx_buffer));
	spi_tx_buffer[SPI_N_BYTES - 1] = CHECKSUM_VALUE;

	// Start timers
	HAL_TIM_Base_Init(&htim2); // debug timer
	HAL_TIM_Base_Start(&htim2);
	timestamp = 0;

	HAL_TIM_Base_Init(&htim3); // debug timer
	HAL_TIM_Base_Start(&htim3);

	// Piezo buzzer initialization
	piezoInit();

#ifdef FIXED_PERIOD
	piezoSetMaxCount(BUZZER_ARR);
	piezoSetRatio(BUZZER_ARR / BUZZER_RATIO - 1);
#endif

	// Led initialization and wake-up sequence
	ledInit(); // uses htim1
	ledSetMaxCount(100);
	for (uint8_t i = 1; i <= 4; i++) {
		piezoSetPSC(freq_list_tim[melodies[0].notes[i - 1]].PSC); // Buzzer follows led sequence

#ifndef FIXED_PERIOD
		piezoSetMaxCount(freq_list_tim[melodies[0].notes[i - 1]].ARR); // Buzzer follows led sequence
		piezoSetRatio(freq_list_tim[melodies[0].notes[i - 1]].ARR / BUZZER_RATIO - 1); // Buzzer follows led sequence
		piezoStart();
#endif

		for (uint8_t j = 0; j < 100; j++) {
			ledSetRatio(j, i);
			HAL_Delay(1);
		}
		for (uint8_t j = 100; j > 0; j--) {
			ledSetRatio(j, i);
			HAL_Delay(1);
		}
		ledSetRatio(100, i);
	}
	piezoSetPSC(0);

	// Super important! We need to wait until the bus is idle, otherwise
	// there is a random shift in the spi_rx_buffer and spi_tx_buffer.
	while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET) {};
	retval = HAL_SPI_TransmitReceive_DMA(&hspi2, spi_tx_buffer, spi_rx_buffer,
	SPI_N_BYTES);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		read_rx_buffer();

		if (buzzer_idx == 0 && state_note_sm != BUZZER_IDLE) {
			state_note_sm = BUZZER_STOP;
		}

		// Monitoring
		// TODO: Upgrade led reactions
		ledSetMaxCount(2000);
		if (last_update_spi > (HAL_GetTick() - 3000)) {
			ledSetRatio((uint16_t) abs(dma_1[0]), 1);
		} else {
			ledSetRatio(0, 1);
		}
		ledSetRatio((uint16_t) abs(dma_1[1]), 2);
		ledSetRatio((uint16_t) abs(dma_3[0]), 3);
		ledSetRatio((uint16_t) abs(dma_3[1]), 4);

		switch (state_note_sm) {
		case BUZZER_IDLE:

			// start condition detection
			if ((buzzer_idx != 0) & (buzzer_idx != buzzer_idx_old)) {
				melody_index = -1;
				for (int i = 0; i < MELODIES_COUNT; i++) {
					if (melodies[i].index == buzzer_idx) {
						melody_index = i;
						break;
					}
				}

				if (melody_index >= 0) {
					note_index = 0;
					buffer_index = 0;
					state_note_sm = BUZZER_PLAY_NEXT;
				}
			}
			buzzer_idx_old = buzzer_idx;

			break;
		case BUZZER_PLAY_NEXT:
			;
			freq_list_t next_note = freq_list_tim[melodies[melody_index].notes[note_index]];
			current_frequency = next_note.f;

			//piezoInit();
			piezoSetPSC(next_note.PSC);
#ifndef FIXED_PERIOD
			piezoSetMaxCount(next_note.ARR); // Buzzer follows led sequence
			piezoSetRatio(next_note.ARR / BUZZER_RATIO - 1); // Buzzer follows led sequence
			piezoStart();
#endif
			memset(mics_f_all, 0x00, sizeof(mics_f_all));

			note_tickstart = HAL_GetTick();
			state_note_sm = BUZZER_WAIT_REC;

			break;
		case BUZZER_WAIT_REC:
			if (note_tickstart + BUZZER_DELAY < HAL_GetTick()) {
				note_tickstart = HAL_GetTick();

				// make sure we don't use the sample from the
				// frequency that played before.
				new_sample_to_process = 0;
				state_note_sm = BUZZER_RECORD;

				// Start acquisition process
				HAL_I2S_Receive_DMA(&hi2s1, (uint16_t*) dma_1, TWO_N_BUFFER);
				HAL_I2S_Receive_DMA(&hi2s3, (uint16_t*) dma_3, TWO_N_BUFFER);

			}
			break;
		case BUZZER_RECORD:
			// we have a new sample to process and want to add it to the buffer
			// need > 1 because we need the two I2S data streams.
			if (new_sample_to_process > 1) {
				flag_fft_processing = 1;
				timestamp = HAL_GetTick();

				// Compute FFT
				// TODO(FD) arm_rfft_fast_f32 may modify mic0, mic1, ...; is this a problem?
				arm_rfft_fast_init_f32(&rfft_instance, N_BUFFER);
				arm_rfft_fast_f32(&rfft_instance, mk1, mk1_f, ifft_flag);
				arm_rfft_fast_init_f32(&rfft_instance, N_BUFFER);
				arm_rfft_fast_f32(&rfft_instance, mk2, mk2_f, ifft_flag);
				arm_rfft_fast_init_f32(&rfft_instance, N_BUFFER);
				arm_rfft_fast_f32(&rfft_instance, mk3, mk3_f, ifft_flag);
				arm_rfft_fast_init_f32(&rfft_instance, N_BUFFER);
				arm_rfft_fast_f32(&rfft_instance, mk4, mk4_f, ifft_flag);

				// for monitoring
				m1  = mk1_f[47*2] * mk1_f[47*2];
				m1 += mk1_f[47*2 + 1] * mk1_f[47*2 + 1];
				m2  = mk2_f[47*2] * mk2_f[47*2];
				m2 += mk2_f[47*2 + 1] * mk2_f[47*2 + 1];
				m3  = mk3_f[47*2] * mk3_f[47*2];
				m3 += mk3_f[47*2 + 1] * mk3_f[47*2 + 1];
				m4  = mk4_f[47*2] * mk4_f[47*2];
				m4 += mk4_f[47*2 + 1] * mk4_f[47*2 + 1];

				flag_fft_processing = 0;

				/* process the data through the Complex Magnitude Module for
				 calculating the magnitude at each bin */
				arm_cmplx_mag_f32(mk2_f, magnitude_mk2, FFTSIZE);


				if (fill_tx_buffer()) {
					new_sample_to_process = 0;

					state_note_sm = BUZZER_WAIT_SEND;
					wait_to_send = 1;
					buzzer_wait_counter = 0;
				}
				else {
					state_note_sm = BUZZER_CHOOSE_NEXT;
				}
			} else {
				// Start acquisition process
				HAL_I2S_Receive_DMA(&hi2s1, (uint16_t*) dma_1, TWO_N_BUFFER);
				HAL_I2S_Receive_DMA(&hi2s3, (uint16_t*) dma_3, TWO_N_BUFFER);
			}

			break;
		case BUZZER_WAIT_SEND:

			if (wait_to_send & (buzzer_wait_counter < BUZZER_WAIT_TIMEOUT)) {
				buzzer_wait_counter++;
				break;
			}

			state_note_sm = BUZZER_CHOOSE_NEXT;
			break;

		case BUZZER_CHOOSE_NEXT:

			// wait until we sent also the last package

			note_index++;
			buffer_index++;

			// point to next element and get its value.
			int16_t frequency_index = melodies[melody_index].notes[note_index];

			if (frequency_index == STOP) {

				state_note_sm = BUZZER_STOP;

			} else if (frequency_index == REPEAT) {
				// go back to beginning of melody
				note_index = 0;
				buffer_index = 0;

				state_note_sm = BUZZER_PLAY_NEXT;
			} else {
				state_note_sm = BUZZER_PLAY_NEXT;
			}
			break;
		case BUZZER_STOP:

			piezoSetPSC(0);

			memset(spi_tx_buffer, 0x00, sizeof(spi_tx_buffer));
			spi_tx_buffer[SPI_N_BYTES - 1] = CHECKSUM_VALUE;
			state_note_sm = BUZZER_IDLE;

			break;
		default:
			break;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1000;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
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
  htim3.Init.Prescaler = 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
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
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 100;
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
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

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
  HAL_GPIO_WritePin(PC3_GPIO_Port, PC3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC3_Pin */
  GPIO_InitStruct.Pin = PC3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(PC3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
#if DCNotchActivated
static inline int16_t DCNotch(uint16_t x, uint8_t filter_index) {
	static int16_t x_prev[4] = { 0, 0, 0, 0 };
	static int16_t y_prev[4] = { 0, 0, 0, 0 };
	if (filter_index == 10) {
		memset(x_prev, 0x00, sizeof(x_prev));
		memset(y_prev, 0x00, sizeof(y_prev));
	}

	y_prev[filter_index] = (((int32_t) y_prev[filter_index] * 0x00007999) >> 16) - x_prev[filter_index] + x;
	x_prev[filter_index] = x;
	return y_prev[filter_index];
}
#endif

void inline process(uint16_t *pIn, float32_t *pOut1, float32_t *pOut2) {
	float32_t window_value;

	// pIn is of size TWO_N_BUFFER, because we have left and right mic.
	// each pOut is of size N_BUFFER.

	// Do not interrupt FFT processing.
	if (flag_fft_processing == 0) {

		for (uint16_t i = 0; i < N_BUFFER; i += 1) {
			if (window_type > 0) {
				window_value = (float32_t) tapering_window[i] / MAX_INT16;
			} else {
				window_value = 1.0;
			}

#if DCNotchActivated
			if (pIn == dma_1) {
				*pOut1++ = (float32_t) DCNotch(*pIn++, 0) / MAX_UINT16 * window_value;
				*pOut2++ = (float32_t) DCNotch(*pIn++, 1) / MAX_UINT16 * window_value;
			} else { // pIn == dma_3
				*pOut1++ = (float32_t) DCNotch(*pIn++, 2) / MAX_UINT16 * window_value;
				*pOut2++ = (float32_t) DCNotch(*pIn++, 3) / MAX_UINT16 * window_value;
			};
#else // not DCNotchActivated
			*pOut1++ = (float32_t) *pIn++ /  MAX_UINT16 * window_value;
			*pOut2++ = (float32_t) *pIn++ /  MAX_UINT16 * window_value;
#endif
		}
		new_sample_to_process++;
	}
}

uint16_t num_to_fill;
uint16_t num_zeros;
uint16_t start_idx;
uint16_t freq_index;
uint32_t freq_index_uint32; // 32 to be compatible with arm
float32_t max_value;

uint16_t min_freq_index;
uint16_t max_freq_index;

// For propeller noise filtering:
uint16_t potential_indices[FFTSIZE]; // will only be partially filled.
uint16_t potential_count;
uint8_t min_fac;
float32_t const prop_factors[N_PROP_FACTORS] = { 0.5, 1, 1.5, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17,
		18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28 };
int counter;
float32_t average_thrust;

void frequency_bin_selection(uint16_t *selected_indices) {

	freq_index = (uint16_t) round(current_frequency / DF);

	// return fixed frequency bins
	if (bin_selection == 3) {
		start_idx = 0;
		if (freq_index >= FFTSIZE_HALF) {
			start_idx = freq_index - FFTSIZE_HALF;
		}
		for (int i = start_idx; i < start_idx + FFTSIZE_SENT + 1; i++) {
			selected_indices[i - start_idx] = i;
		}
		return;
	}

	// This happens in the beginning only, afterwards it only happens if there
	// was faulty communication between the Crazyflie and Audio deck.
	if (min_freq >= max_freq) {
		return;
	}

	// TODO(FD): for the moment we just use the average thrust here. We could
	// also loop through all propeller frequencies and not use any of them,
	// that would be more appropriate if the thrust values vary a lot between motors.
	if (filter_prop_enable) {
		average_thrust = 0;
		counter = 0; // used to average only non-zero motors (e.g. for 1-motor experiments)

		// make sure to reset, otherwise we keep filtering out propellers noise
		// even when they are not turning anymore.
		prop_freq = 0;
		for (int i = 0; i < N_MOTORS; i++) {
			if (param_array[i] > 0) {
				average_thrust += (float32_t) param_array[i];
				counter++;
			}
		}
		// This was the old and potentially more general formula in general,
		// however it can be simplified for the range of thrust values of interest,
		// which is between 45000 and 55000:
		// prop_freq = 3.27258551 * sqrt(average_thrust) - 26.41814899;
		//
		// New formula:
		// f(45000) = 671, f(55000) = 750
		// prop_freq = (750 - 671) / (55000 - 45000) * (thrust - 45000) + 671
		// 0.0079 = (750 - 671) / (55000 - 45000), 315.5 = 671 - 0.0079 * 45000
		if (counter > 0) {
			average_thrust /= counter;
			prop_freq = 315.5 + 0.0079 * average_thrust;

		}
	}

	min_freq_index = (uint16_t) round(min_freq / DF);
	max_freq_index = (uint16_t) round(max_freq / DF);

	// Fill bin candidates with the available ones, after removing propeller frequencies.
	min_fac = 0;
	potential_count = 0;

	// TODO(FD): ideally, below should be i <= max_freq_index, but we leave it like this for consistency with previous measurements.
	for (int i = min_freq_index; i < max_freq_index; i++) {
		bool use_this = 1;
		if (filter_prop_enable) {
			for (uint8_t j = min_fac; j < N_PROP_FACTORS; j++) {
				if (fabs(((i * DF) - (prop_factors[j] * prop_freq))) < delta_freq) {
					use_this = false;

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

	num_to_fill = min(potential_count, FFTSIZE_SENT);
	num_zeros = FFTSIZE_SENT - num_to_fill;
	start_idx = 0;

	if (bin_selection == 0) {
		float32_t decimation = (float32_t) potential_count / num_to_fill;
		for (int i = 0; i < num_to_fill; i++) {
			selected_indices[i] = potential_indices[(int) round(i * decimation)];
		}
		// what above corresponds to if potential_count <= FFTSIZE_SENT:
		//memcpy(selected_indices, potential_indices, num_to_fill * INT16_PRECISION);
	}

	else {
		start_idx = 0;
		// Put the buzzer frequency in the first bin, if requested
		if (bin_selection == 2) {
			selected_indices[0] = freq_index;
			start_idx = 1;
		}
		// Put the sorted frequencies in the next bins
		if (bin_selection >= 1) {
			struct index_amplitude sort_this[potential_count];

			for (int i = 0; i < potential_count; i++) {
				sort_this[i].amplitude = magnitude_mk2[potential_indices[i]];
				sort_this[i].index = potential_indices[i];
			}

			qsort(sort_this, potential_count, sizeof(sort_this[0]), compare_amplitudes);
			for (int i = start_idx; i < num_to_fill; i++) {
				selected_indices[i] = sort_this[i - start_idx].index;
			}
		}
	}

	// Fill remaining bins with zeros
	memset(&selected_indices[num_to_fill], 0x00, INT16_PRECISION * num_zeros);
}

uint8_t fill_tx_buffer() {
	// The buffer will be filled like
	// [m1_real[0], m2_real[0], m3_real[0], m4_real[0], m1_imag[0], m2_imag[0], m3_imag[0], m4_imag[0],
	//  m1_real[1], m2_real[1], m3_real[1], m4_real[1], m1_imag[1], m2_imag[1], m3_imag[1], m4_imag[1],
	//  ...
	//  m1_real[N], m2_real[N], m3_real[N], m4_real[N], m1_imag[N], m2_imag[N], m3_imag[N], m4_imag[N]]
	//  where N is FFTSIZE_SENT-1


	if (bin_selection < 5) {
		// MODE "32 Bins, with selection schemes"

		frequency_bin_selection(selected_indices);

		// TODO(FD) the plan here was to do an averaging, but we need to
		// decompose in magnitude and phase to do this, which is currently
		// out of the scope of this application. So instead we use the latest
		// value.

		i_array = 0;
		for (int i_fbin = 0; i_fbin < FFTSIZE_SENT; i_fbin++) {
			mics_f_all[i_array++] = mk1_f[N_COMPLEX * selected_indices[i_fbin]];
			mics_f_all[i_array++] = mk2_f[N_COMPLEX * selected_indices[i_fbin]];
			mics_f_all[i_array++] = mk3_f[N_COMPLEX * selected_indices[i_fbin]];
			mics_f_all[i_array++] = mk4_f[N_COMPLEX * selected_indices[i_fbin]];
			mics_f_all[i_array++] = mk1_f[N_COMPLEX * selected_indices[i_fbin] + 1];
			mics_f_all[i_array++] = mk2_f[N_COMPLEX * selected_indices[i_fbin] + 1];
			mics_f_all[i_array++] = mk3_f[N_COMPLEX * selected_indices[i_fbin] + 1];
			mics_f_all[i_array++] = mk4_f[N_COMPLEX * selected_indices[i_fbin] + 1];
		}

		// Fill buffer with audio data
		memcpy(spi_tx_buffer, mics_f_all, sizeof(mics_f_all));
		i_array = sizeof(mics_f_all);

		// Fill with bins indices
		memcpy(&spi_tx_buffer[i_array], selected_indices, sizeof(selected_indices));
		i_array += sizeof(selected_indices);

		// Fill with timestamp
		memcpy(&spi_tx_buffer[i_array], &timestamp, sizeof(timestamp));
		i_array += sizeof(timestamp);

		assert(i_array == SPI_N_BYTES - 1);
		spi_tx_buffer[SPI_N_BYTES - 1] = CHECKSUM_VALUE;

		// now we wait until this package is sent to play next note.
		return 1;
	} else {
		// MODE "ONE BUFFER, ONE SWEEP"

		// set the CHECKSUM to 0 so that if we communicate during filling, the package is not valid.
		spi_tx_buffer[SPI_N_BYTES - 1] = 0;

		freq_index = (uint16_t) round(current_frequency / DF);

		// Calculate current frequency of interest
		if (bin_selection == 6) {
			min_freq_index = max(freq_index - WINDOW_FREQS, 0);
			max_freq_index = min(freq_index + WINDOW_FREQS, FFTSIZE-1);

			arm_max_f32(
					&magnitude_mk2[min_freq_index],
					max_freq_index - min_freq_index - 1,
					&max_value,
					&freq_index_uint32
			);
			// no need to check for overflow because freq_index is smaller than FFTSIZE /2.
			freq_index = freq_index_uint32 + min_freq_index;
		}

		// Variable for memory indexing
		i_array = buffer_index * N_MICS * N_COMPLEX * FLOAT_PRECISION;

		// Fill buffer with audio data
		memcpy(&spi_tx_buffer[i_array], &mk1_f[N_COMPLEX * freq_index], FLOAT_PRECISION);
		i_array += FLOAT_PRECISION;
		memcpy(&spi_tx_buffer[i_array], &mk2_f[N_COMPLEX * freq_index], FLOAT_PRECISION);
		i_array += FLOAT_PRECISION;
		memcpy(&spi_tx_buffer[i_array], &mk3_f[N_COMPLEX * freq_index], FLOAT_PRECISION);
		i_array += FLOAT_PRECISION;
		memcpy(&spi_tx_buffer[i_array], &mk4_f[N_COMPLEX * freq_index], FLOAT_PRECISION);
		i_array += FLOAT_PRECISION;
		memcpy(&spi_tx_buffer[i_array], &mk1_f[N_COMPLEX * freq_index + 1], FLOAT_PRECISION);
		i_array += FLOAT_PRECISION;
		memcpy(&spi_tx_buffer[i_array], &mk2_f[N_COMPLEX * freq_index + 1], FLOAT_PRECISION);
		i_array += FLOAT_PRECISION;
		memcpy(&spi_tx_buffer[i_array], &mk3_f[N_COMPLEX * freq_index + 1], FLOAT_PRECISION);
		i_array += FLOAT_PRECISION;
		memcpy(&spi_tx_buffer[i_array], &mk4_f[N_COMPLEX * freq_index + 1], FLOAT_PRECISION);
		i_array += FLOAT_PRECISION;

		// Fill with bins indices
		memcpy(&spi_tx_buffer[AUDIO_N_BYTES + buffer_index * sizeof(freq_index)], &freq_index, sizeof(freq_index));

		// Fill with timestamp
		memcpy(&spi_tx_buffer[AUDIO_N_BYTES + FBINS_N_BYTES], &timestamp, sizeof(timestamp));

		// Activate Checksum only if sweep is completed
		if (buffer_index == melodies[melody_index].length - 1) {
			buffer_index = -1;

			spi_tx_buffer[SPI_N_BYTES - 1] = CHECKSUM_VALUE;

			// wait for this to be sent.
			return 1;
		}
		return 0;
	}
}

void read_rx_buffer() {
	memcpy(param_array, spi_rx_buffer, sizeof(param_array));

	// Sometimes, because of faulty communication, the packet is broken and we get
	// impossible values for the parameters. In that case they should not be updated.
	if (param_array[N_MOTORS] >= param_array[N_MOTORS + 1])
		return;

	min_freq = param_array[N_MOTORS];
	max_freq = param_array[N_MOTORS + 1];
	buzzer_idx = param_array[N_MOTORS + 2];
	delta_freq = param_array[N_MOTORS + 3];
	n_average = param_array[N_MOTORS + 4];
	filter_prop_enable = param_array[N_MOTORS + 5];
	bin_selection = param_array[N_MOTORS + 6];

	// initialize everything if we have changed window.
	if (param_array[N_MOTORS + 7] != window_type) {
		window_type = param_array[N_MOTORS + 7];
		switch (window_type) {
		case 1:
			tapering_window = hann_window;
			break;
		case 2:
			tapering_window = flattop_window;
			break;
		case 3:
			tapering_window = tukey_window;
			break;
		default:
			break;
		}
		// reset the DC notch filter
#if DCNotchActivated
		DCNotch(0, 10);
#endif
	}
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
