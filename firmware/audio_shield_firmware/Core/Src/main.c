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
#define nMic 4
#define FFTSIZE_SENT 32
#define ARRAY_SIZE nMic*FFTSIZE_SENT*4*2

uint32_t ifftFlag = 0;
uint32_t doBitReverse = 1;

uint32_t time_fft;
volatile uint32_t time_bin_process;

uint8_t processing = 0;
uint8_t new_sample_to_send = 0;

#ifdef MATRIX_INVERSE_CALCULATION

arm_matrix_instance_f32 matXf;
arm_matrix_instance_f32 matXfh;
arm_matrix_instance_f32 matR[FFTSIZE];
arm_matrix_instance_f32 matRinv[FFTSIZE];

float vect_Xf[nMic * 2];
float vect_Xfh[nMic * 2];
float vect_R[FFTSIZE][nMic * nMic * 2];
float vect_Rinv[FFTSIZE][nMic * nMic * 2];

// TESTING FD
//arm_matrix_instance_f32 matX;
//arm_matrix_instance_f32 matXh;
//arm_matrix_instance_f32 result;

#else

float mat_Xf[FFTSIZE][nMic * 2];
uint8_t mat_Xf_bytes[FFTSIZE_SENT][nMic * 2][4];
uint8_t array_Xf_bytes[ARRAY_SIZE];

#endif

uint8_t srcRows;
uint8_t srcColumns;

arm_rfft_fast_instance_f32 S;

arm_status status;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;

I2S_HandleTypeDef hi2s1;
I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi3_rx;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S3_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S1_Init(void);
static void MX_SPI2_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */

void Process(int16_t *pIn, float *pOut1, float *pOut2, uint16_t size);
void corr_matrix_to_corr_array(uint8_t byte_matrix[FFTSIZE_SENT][nMic * 2][4],
		uint8_t byte_array[ARRAY_SIZE]);
void float_to_byte_array(float input, uint8_t output[]);
void float_matrix_to_byte_matrix(float float_matrix[FFTSIZE_SENT][nMic * 2],
		uint8_t byte_matrix[FFTSIZE_SENT][nMic * 2][4]);
void send_corr_matrix();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {

}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c){
	STOPCHRONO;
	time_fft = time_us;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c){

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
	MX_I2C1_Init();
	MX_I2S1_Init();
	MX_SPI2_Init();
	MX_UART4_Init();
	/* USER CODE BEGIN 2 */

	// Start DMAs
	HAL_I2S_Receive_DMA(&hi2s1, (uint16_t*) dma_1, FULL_BUFFER_SIZE);
	HAL_I2S_Receive_DMA(&hi2s3, (uint16_t*) dma_3, FULL_BUFFER_SIZE);

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
#endif

		//HAL_Delay(100);
		if (new_sample_to_send) {
			processing = 1;
			//STOPCHRONO;
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

#ifndef MATRIX_INVERSE_CALCULATION

			// Matrix initialisation
			//srcRows = nMic;
			//srcColumns = 1;
			//arm_mat_init_f32(&matXf, srcRows, srcColumns, vect_Xf);

			// Frequency bin processing
			uint16_t g = 0;
			for (int f = 0; f < FFTSIZE; f += 2) {

				/*
				 * Xf = [mic1_f_real, mic1_f_imag,
				 * 		 mic2_f_real, mic2_f_imag,
				 * 		 mic3_f_real, mic3_f_imag,
				 * 		 mic4_f_real, mic4_f_imag ]
				 */
				mat_Xf[g][0] = left_1_f[f];
				mat_Xf[g][1] = left_3_f[f];
				mat_Xf[g][2] = right_1_f[f];
				mat_Xf[g][3] = right_3_f[f];
				mat_Xf[g][4] = left_1_f[f + 1];
				mat_Xf[g][5] = left_3_f[f + 1];
				mat_Xf[g][6] = right_1_f[f + 1];
				mat_Xf[g][7] = right_3_f[f + 1];
				g++;
			}

			STOPCHRONO;
			send_corr_matrix(mat_Xf);
			new_sample_to_send = 0;
		}
#else

		// Matrix initialisation
		srcRows = nMic;
		srcColumns = 1;
		arm_mat_init_f32(&matXf, srcRows, srcColumns, vect_Xf);


		srcRows = 1;
		srcColumns = nMic;
		arm_mat_init_f32(&matXfh, srcRows, srcColumns, vect_Xfh);

		uint16_t f = 0;


		float vect_Rf_real_inv[nMic*nMic];
		float vect_Rf_imag_inv[nMic*nMic];
		arm_matrix_instance_f32 mat_Rf_real_inv;
		arm_matrix_instance_f32 mat_Rf_imag_inv;

		float vect_Rf_real[nMic*nMic];
		float vect_Rf_imag[nMic*nMic];
		arm_matrix_instance_f32 mat_Rf_real;
		arm_matrix_instance_f32 mat_Rf_imag;

		float vect_interm_1[nMic*nMic];
		float vect_interm_2[nMic*nMic];
		arm_matrix_instance_f32 mat_interm_1;
		arm_matrix_instance_f32 mat_interm_2;

		// Frequency bin processing
		for (f = 0; f < FFTSIZE; f += 2) {


			/*
			 * Xf = [mic1_f_real, mic1_f_imag,
			 * 		 mic2_f_real, mic2_f_imag,
			 * 		 mic3_f_real, mic3_f_imag,
			 * 		 mic4_f_real, mic4_f_imag ]
			 */
			vect_Xf[0] = left_1_f[f];
			vect_Xf[1] = left_1_f[f + 1];
			vect_Xf[2] = left_3_f[f];
			vect_Xf[3] = left_3_f[f + 1];
			vect_Xf[4] = right_1_f[f];
			vect_Xf[5] = right_1_f[f + 1];
			vect_Xf[6] = right_3_f[f];
			vect_Xf[7] = right_3_f[f + 1];

			/*
			 * Xfh = Xfconj
			 */
			arm_cmplx_conj_f32(vect_Xf, vect_Xfh, nMic);


			srcRows = nMic;
			srcColumns = nMic;
			arm_mat_init_f32(&matR[f], srcRows, srcColumns, vect_R[f]);


			// TESTING FD
			/* square example

			float vectresult[8];

			//float vectX[4] = {1, -1, 1, 1};
			//float vectXh[4] = {1, 1, 1, -1};
			//works ok: 2 0 0 -2 0 2 2 0

			//float vectX[4] = {1, 0, -1, 0};
			//float vectXh[4] = {1, 0, 1, 0};
			//works ok: 1 0 1 0 -1 0 -1 0

			srcRows = 2;
			srcColumns = 1;
			arm_mat_init_f32(&matX, srcRows, srcColumns, vectX);
			srcRows = 1;
			srcColumns = 2;
			arm_mat_init_f32(&matXh, srcRows, srcColumns, vectXh);
			srcRows = 2;
			srcColumns = 2;
			arm_mat_init_f32(&result, srcRows, srcColumns, vectresult);
			status = arm_mat_cmplx_mult_f32(&matX, &matXh, &result);
			*/

			/* rectangular example
			float vectresult[12];
			float vectX[6] = {1, -1, 1, 1, 1, -1};
			float vectXh[4] = {1, 1, 1, -1};
			//works ok: 2 0 0 -2 0 2 2 0 2 0 0 -2

			srcRows = 3;
			srcColumns = 1;
			arm_mat_init_f32(&matX, srcRows, srcColumns, vectX);
			srcRows = 1;
			srcColumns = 2;
			arm_mat_init_f32(&matXh, srcRows, srcColumns, vectXh);
			srcRows = 3;
			srcColumns = 2;
			arm_mat_init_f32(&result, srcRows, srcColumns, vectresult);
			status = arm_mat_cmplx_mult_f32(&matX, &matXh, &result);
			*/
			// TESTING FD

			/*
			 * R = Xf*Xfconj
			 */
			status = arm_mat_cmplx_mult_f32(&matXf, &matXfh, &matR[f]);

#define LAMBDA 0.01

			/*
			 * Rf_real = real(R)
			 * Rf_imag = imag(R)
			 */
			for(uint8_t i = 0; i < nMic*nMic; i ++){
				vect_Rf_real[i] = vect_R[f][2*i];
				vect_Rf_imag[i] = vect_R[f][2*i+1];
			}

			for(uint8_t i = 0; i < nMic; i++){
				vect_Rf_real[i*nMic+i] += LAMBDA;
				vect_Rf_imag[i*nMic+i] += LAMBDA;
			}

			arm_mat_init_f32(&mat_Rf_real, nMic, nMic, vect_Rf_real);
			arm_mat_init_f32(&mat_Rf_imag, nMic, nMic, vect_Rf_imag);
			arm_mat_init_f32(&mat_Rf_real_inv, nMic, nMic, vect_Rf_real_inv);
			arm_mat_init_f32(&mat_Rf_imag_inv, nMic, nMic, vect_Rf_imag_inv);
			arm_mat_init_f32(&mat_interm_1, nMic, nMic, vect_interm_1);
			arm_mat_init_f32(&mat_interm_2, nMic, nMic, vect_interm_2);

			//Compute A^-1 and B^-1
			arm_mat_inverse_f32(&mat_Rf_real, &mat_Rf_real_inv);
			arm_mat_inverse_f32(&mat_Rf_imag, &mat_Rf_imag_inv);

			/*
			 * Revive data
			 * Rf_real = real(R)
			 * Rf_imag = imag(R)
			 */
			for(uint8_t i = 0; i < nMic*nMic; i ++){
				vect_Rf_real[i] = vect_R[f][2*i];
				vect_Rf_imag[i] = vect_R[f][2*i+1];
			}

			for(uint8_t i = 0; i < nMic; i++){
				vect_Rf_real[i*nMic+i] += LAMBDA;
				vect_Rf_imag[i*nMic+i] += LAMBDA;
			}

			/*
			 * Rinv = R^-1 = (A + B*A^-1*B)^-1 - i*(B + A*B^-1*A)^-1
			 */

			srcRows = nMic;
			srcColumns = nMic;
			arm_mat_init_f32(&matRinv[f], srcRows, srcColumns, vect_Rinv[f]);

			// interm_1 = B*A^-1
			arm_mat_mult_f32(&mat_Rf_imag, &mat_Rf_real_inv, &mat_interm_1);

			// interm_2 = interm_1*B = B*A^-1*B
			arm_mat_mult_f32(&mat_interm_1, &mat_Rf_imag, &mat_interm_2);

			// interm_1 = A+interm_2 = A + B*A^-1*B
			arm_mat_add_f32(&mat_Rf_real, &mat_interm_2, &mat_interm_1);

			// interm_2 = interm_1^-1 =(A + B*A^-1*B)^-1
			// this is the real part of the result
			arm_mat_inverse_f32(&mat_interm_1, &mat_interm_2);

			for(uint8_t i = 0; i < nMic*nMic; i ++){
				vect_Rinv[f][2*i] = vect_interm_2[i];
			}

			// interm_3 = A*B^-1
			arm_mat_mult_f32(&mat_Rf_real, &mat_Rf_imag_inv, &mat_interm_1);

			// interm_4 = interm_3*A = A*B^-1*A
			arm_mat_mult_f32(&mat_interm_1, &mat_Rf_real, &mat_interm_2);

			// interm_3 = B+interm_4 = B + A*B^-1*A
			arm_mat_add_f32(&mat_Rf_imag, &mat_interm_2, &mat_interm_1);

			// interm_4 = interm_3^-1 =(B + A*B^-1*A)^-1
			// this is the imag part of the result
			arm_mat_inverse_f32(&mat_interm_1, &mat_interm_2);

			//arm_mat_inverse_f32(&matR[f], &matRinv[f]);

			for(uint8_t i = 0; i < nMic*nMic; i ++){
				vect_Rinv[f][2*i+1] = -vect_interm_2[i];
			}

			STOPCHRONO;
			time_bin_process = time_us;
		}

#ifdef FFT_FIND_PEAK
		/* Calculating the magnitude at each bin */
		arm_cmplx_mag_f32(left_1, testOutput, FFTSIZE);

		/* Calculates maxValue and returns corresponding BIN value */
		testOutput[0] = 0;
		arm_max_f32(testOutput, FFTSIZE, &maxValue, &testIndex);

		processing = 0;

#endif

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
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 94;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

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
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_HARD_INPUT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
 * @brief UART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART4_Init(void) {

	/* USER CODE BEGIN UART4_Init 0 */

	/* USER CODE END UART4_Init 0 */

	/* USER CODE BEGIN UART4_Init 1 */

	/* USER CODE END UART4_Init 1 */
	huart4.Instance = UART4;
	huart4.Init.BaudRate = 115200;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart4) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN UART4_Init 2 */

	/* USER CODE END UART4_Init 2 */

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
	/* DMA1_Stream6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
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

/*
 void inline Process_3(int16_t *pIn, uint16_t size) {
 for (uint16_t i = 0; i < size/2; i++){
 left_3[i] = *pIn++;
 right_3[i] = *pIn++;
 }
 }
 */
void send_corr_matrix() {
	float_matrix_to_byte_matrix(mat_Xf, mat_Xf_bytes);
	corr_matrix_to_corr_array(mat_Xf_bytes, array_Xf_bytes);
	HAL_I2C_Slave_Transmit_DMA(&hi2c1, array_Xf_bytes, ARRAY_SIZE);
}

void corr_matrix_to_corr_array(uint8_t byte_matrix[FFTSIZE_SENT][nMic * 2][4],
		uint8_t byte_array[ARRAY_SIZE]) {
	uint16_t index = 0;
	for (int i = 0; i < FFTSIZE_SENT; i++) {
		for (int j = 0; j < nMic * 2; j++) {
			for (int k = 0; k < 4; k++) {
				byte_array[index] = byte_matrix[i][j][k];
				index++;
			}
		}
	}
}

void float_matrix_to_byte_matrix(float float_matrix[FFTSIZE_SENT][nMic * 2],
		uint8_t byte_matrix[FFTSIZE_SENT][nMic * 2][4]) {
	for (int i = 0; i < FFTSIZE_SENT; i++) {
		for (int j = 0; j < nMic * 2; j++) {
			float_to_byte_array(float_matrix[i][j], byte_matrix[i][j]);
		}
	}
}

void float_to_byte_array(float input, uint8_t output[]) {
	uint32_t temp = *((uint32_t*) &input);
	for (int i = 0; i < 4; i++) {
		output[i] = temp & 0xFF;
		temp >>= 8;
	}
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
