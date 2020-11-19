/**
  ******************************************************************************
  * @file    xnucleoihm02a1_interface.c
  * @brief   This file is used as interface between the 
  *          X-NUCLEO-IHM02A1 and the NUCLEO-F4xx board.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2014 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "xnucleoihm02a1_interface.h"

/**
  * @addtogroup BSP
  * @{
  */

/**
  * @addtogroup X-NUCLEO-IHM02A1
  * @{
  */

/**
  * @addtogroup NUCLEO_Interface
  * @{
  */

/**
  * @addtogroup NUCLEO_Exported_Variables
  * @{
  */

/**
  * @brief  The data structure for all further instances to ADC.
  */
ADC_HandleTypeDef hadc1;
/**
  * @brief  The data structure for all further instances to SPI1.
  */
SPI_HandleTypeDef hspi1;
/**
  * @brief  The data structure for all further instances to SPI2.
  */
SPI_HandleTypeDef hspi2;

/**
  * @}
  */ /* End of NUCLEO_Exported_Variables */

/**
  * @defgroup   NUCLEO_Private_Functions
  * @brief      NUCLEO Private Functions.
  * @{
  */

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_ADC1_Init(void);
void MX_SPI_Init(void);
void MX_SPI1_Init(void);
void MX_SPI2_Init(void);
void MX_USART2_Init(void);

/**
  * @}
  */ /* End of NUCLEO_Private_Functions */

/**
  * @addtogroup NUCLEO_Private_Functions
  * @{
  */

/**
  * @brief  This function configures the System Clock
  * 
  * @note   The System Clock will be configured as following:
  *         - PLL Source: HSI
  *         - SYSCLK: 84 MHz
  *         - HCLK: 84 MHz
  *         - APB1 Peripheral Clocks: 42 MHz
  *         - APB1 Timer Clocks: 84 MHz
  *         - APB2 Peripheral Clocks: 84 MHz
  *         - APB2 Timer Clocks: 84 MHz
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

}

/**
  * @brief  This function initializes the GPIO MX.
  */
void MX_GPIO_Init(void)
{
#ifdef NUCLEO_USE_USER_BUTTON
  /* Configures Button GPIO and EXTI Line */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
#endif

#ifdef NUCLEO_USE_USER_LED
  /* Configures LED GPIO */
  BSP_LED_Init(LED2);
#endif
}

/**
  * @brief  Initialize the SPI used by the NUCLEO board.
  *
  * @note   It selects the @ref MX_SPI1_Init or @ref MX_SPI2_Init
  *         related to the defined macro @ref NUCLEO_USE_SPI_1 or @ref NUCLEO_USE_SPI_2.
  */
void MX_SPI_Init(void)
{
#ifdef NUCLEO_USE_SPI_1
  MX_SPI1_Init();
#endif
#ifdef NUCLEO_USE_SPI_2
  MX_SPI2_Init();
#endif
}

/**
  * @brief  This function initializes the SPI1 MX
  *
  * @note   It sets the <i>hspi1</i> data structure for all further instances to
  *         SPI1
  *
  * @note   The SPI1 peripheral is configured as following:
  *         - Full-Duplex Master
  *         - 8-Bits
  *         - CPOL High
  *         - CPHA 2nd Edge
  *         - Baud Rate lower than 5 MBits/s
  */
void MX_SPI1_Init(void)
{
  #define MAX_BAUDRATE  5000000
  uint32_t freq;
  uint16_t freq_div;
  uint32_t spi_baudrateprescaler;
  
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  
  freq = HAL_RCC_GetPCLK2Freq();
  freq_div = (freq / MAX_BAUDRATE);
  
  if (freq_div < 2)
  {
    spi_baudrateprescaler = SPI_BAUDRATEPRESCALER_2;
  }
  else
  {
    if (freq_div < 4)
    {
      spi_baudrateprescaler = SPI_BAUDRATEPRESCALER_4;
    }
    else
    {
      if (freq_div < 8)
      {
        spi_baudrateprescaler = SPI_BAUDRATEPRESCALER_8;
      }
      else
      {
        if (freq_div < 16)
        {
          spi_baudrateprescaler = SPI_BAUDRATEPRESCALER_16;
        }
        else
        {
          if (freq_div < 32)
          {
            spi_baudrateprescaler = SPI_BAUDRATEPRESCALER_32;
          }
          else
          {
            if (freq_div < 64)
            {
              spi_baudrateprescaler = SPI_BAUDRATEPRESCALER_64;
            }
            else
            {
              if (freq_div < 128)
              {
                spi_baudrateprescaler = SPI_BAUDRATEPRESCALER_128;
              }
              else
              {
                if (freq_div < 256)
                {
                  spi_baudrateprescaler = SPI_BAUDRATEPRESCALER_256;
                }
                else
                {
                  /* the condition is not possible, you should reduce the CPU frequency */
                  while(1);
                }
              }
            }
          }
        }
      }
    }
  }
  
  hspi1.Init.BaudRatePrescaler = spi_baudrateprescaler;  // the baudrate will be lower than MAX_BAUDRATE (5 MBits/s)
  HAL_SPI_Init(&hspi1);
}

/**
  * @brief  This function initializes the SPI2 MX
  *
  * @note   It sets the <i>hspi2</i> data structure for all further instances to
  *         SPI2
  *
  * @note   The SPI2 peripheral is configured as following:
  *         - Full-Duplex Master
  *         - 8-Bits
  *         - CPOL High
  *         - CPHA 2nd Edge
  *         - Baud Rate lower than 5 MBits/s
  */
void MX_SPI2_Init(void)
{
  #define MAX_BAUDRATE  5000000
  uint32_t freq;
  uint16_t freq_div;
  uint32_t spi_baudrateprescaler;
  
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  
  freq = HAL_RCC_GetPCLK1Freq();
  freq_div = (freq / MAX_BAUDRATE);
  
  if (freq_div < 2)
  {
    spi_baudrateprescaler = SPI_BAUDRATEPRESCALER_2;
  }
  else
  {
    if (freq_div < 4)
    {
      spi_baudrateprescaler = SPI_BAUDRATEPRESCALER_4;
    }
    else
    {
      if (freq_div < 8)
      {
        spi_baudrateprescaler = SPI_BAUDRATEPRESCALER_8;
      }
      else
      {
        if (freq_div < 16)
        {
          spi_baudrateprescaler = SPI_BAUDRATEPRESCALER_16;
        }
        else
        {
          if (freq_div < 32)
          {
            spi_baudrateprescaler = SPI_BAUDRATEPRESCALER_32;
          }
          else
          {
            if (freq_div < 64)
            {
              spi_baudrateprescaler = SPI_BAUDRATEPRESCALER_64;
            }
            else
            {
              if (freq_div < 128)
              {
                spi_baudrateprescaler = SPI_BAUDRATEPRESCALER_128;
              }
              else
              {
                if (freq_div < 256)
                {
                  spi_baudrateprescaler = SPI_BAUDRATEPRESCALER_256;
                }
                else
                {
                  /* the condition is not possible, you should reduce the CPU frequency */
                  while(1);
                }
              }
            }
          }
        }
      }
    }
  }
  
  hspi2.Init.BaudRatePrescaler = spi_baudrateprescaler; // the baudrate will be lower than MAX_BAUDRATE (5 MBits/s)
  HAL_SPI_Init(&hspi2);
}

/**
  * @brief  This function initializes the USART2 MX.
  * @note   It sets the <i>huart2</i> data structure for all further instances
  *         to USART2.
  * @note   The USART2 peripheral is configured as following:
  *         - Baud Rate:  115200
  *         - Data Bits:  8
  *         - Stop Bit:   1
  *         - Parity:     None
  *         - Mode:       TX/RX
  */
void MX_USART2_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);
}

/**
  * @brief  This function initializes the ADC MX
  *
  * @note   It sets the <i>hadc</i> data structure for all further instances to ADC
*/
void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;

  /* GPIO Ports Clock Enable */
  __GPIOB_CLK_ENABLE();

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION12b;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = EOC_SINGLE_CONV;
  HAL_ADC_Init(&hadc1);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

}

/**
  * @}
  */ /* End of NUCLEO_Private_Functions */

/**
  * @addtogroup NUCLEO_Exported_Functions
  * @{
  */

/**
  * @brief  This function initializes some peripherals of the NUCELO board
  *         (HAL, Clock, NVIC, LED and user button)
  */
void NUCLEO_Board_Init(void)
{
  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  
  /* Initialize the SPI used by the X-NUCLEO-IHM02A1 */
  MX_SPI_Init();
  
#ifdef NUCLEO_USE_USART
  /* Initialize the USART peripheral */
  MX_USART2_Init();
#endif  

#ifdef NUCLEO_USE_USER_LED
  /* Perform 3 repetition of blinking user LED at 50% duty cycle with 250 ms as period */
  User_LED_Blinking(3, 750);
#endif
}

/**
  * @brief  Blinking user LED at 50% duty cycle.
  * @param  repetitions The number of  repetions.
  * @param  period_ms   The blinking period in ms.
  */
void User_LED_Blinking(uint8_t repetitions, uint16_t period_ms)
{
  uint8_t r;
  uint16_t half_period_ms;
  
  half_period_ms = period_ms >> 1;
  
  for (r=0; r<repetitions; r++)
  {
    /* Switch on the user LED */
    BSP_LED_On(LED2);
    /* ms delay */
    HAL_Delay(half_period_ms);
    /* Switch off the user LED */
    BSP_LED_Off(LED2);
    /* ms delay */
    HAL_Delay(half_period_ms);
  }
}

/**
  * @}
  */ /* End of NUCLEO_Exported_Functions */

/**
  * @}
  */ /* End of NUCLEO_Interface */

/**
  * @}
  */ /* End of X-NUCLEO-IHM02A1 */

/**
  * @}
  */ /* End of BSP */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
