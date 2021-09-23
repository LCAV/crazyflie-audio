/**
  ******************************************************************************
  * @file       example_usart.h
  * @date       01/10/2014 12:00:00
  * @brief      This file provides the function prototypes to send commands to
  *             the L6470 via Nucleo USART 
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EXAMPLE_USART_H
#define __EXAMPLE_USART_H

#ifdef __cplusplus
 extern "C" {
#endif 

#include "xnucleoihm02a1_interface.h"

/**
  * @addtogroup MicrosteppingMotor_Example
  * @{
  */

/**
  * @defgroup   ExampleUsart
  * @brief      To manage the USART communication between a PC and the STM32NUCLEO
  *             related to the X-NUCLEO-IHM02A1.
  * @{
  */

/**
  * @addtogroup ExampleUsartExportedConstants
  * @{
  */
   
#define DECODE_KEYBOARD_REMOTE 1

#if DECODE_KEYBOARD_REMOTE

#define USARTTEXTSTRINGSIZE  1 //!< Max number of characters can be entered from the PC.

#else

#define USARTTEXTSTRINGSIZE  64 //!< Max number of characters can be entered from the PC.

#endif

/**
  * @}
  */ /* End of ExampleUsartExportedConstants */

/**
  * @addtogroup ExampleUsartExportedTypes
  * @{
  */
   
/**
  * @brief  The structure used to store the mnemonic names of the L6470 and its
  *         related motor inside the daisy chain.
  */
typedef struct {
  uint8_t L6470IdMnemonic[9];   //!< Mnemonic L6470 Identifier
  uint8_t MotorIdMnemonic[4];   //!< Mnemonic Motor Identifier
} sL6470_DaisyChainMnemonic;

/**
  * @brief Structure to store the command bundle in text format
  */
typedef struct {
  uint8_t MotorName[3];     //!< L6470 mnemonic
  uint8_t CommandName[12];  //!< command mnemonic
  uint8_t Param[3][11];     //!< parameters of command
} sL6470_TextCommandBundle;

/**
  * @brief The hexdecimal format to be used
  */
typedef enum
{
  HALFBYTE_F,     //!< 1 hex digit 
  BYTE_F,         //!< 2 hex digits
  WORD_F,         //!< 4 hex digits
  DOUBLEWORD_F    //!< 8 hex digits
}eHexFormat;

/**
  * @}
  */ /* End of ExampleUsartExportedTypes */

/**
  * @defgroup   ExampleUsartExportedVariables
  * @{
  */

extern sL6470_DaisyChainMnemonic L6470_DaisyChainMnemonic[];

/**
  * @}
  */ /* End of ExampleUsartExportedVariables */

/**
  * @defgroup   ExampleUsartExportedFunctions
  * @{
  */

void USART_TxWelcomeMessage(void);
void USART_Transmit(UART_HandleTypeDef* huart, uint8_t* TextString);
void USART_CheckAppCmd(void);
void USART_ITCharManager(UART_HandleTypeDef* huart);
void USART_PrintRegisterValues(uint8_t ExpBrd, uint8_t L6470_Id);
void USART_PrintAllRegisterValues(void);
void Fill_L6470_DaisyChainMnemonic(void);
void num2str(uint32_t nbr, uint8_t *str);
uint8_t* num2hex(uint32_t num, eHexFormat HexFormat);

/**
  * @}
  */ /* End of ExampleUsartExportedFunctions */

/**
  * @}
  */ /* End of ExampleUsart */

/**
  * @}
  */ /* End of MicrosteppingMotor_Example */

#ifdef __cplusplus
}
#endif

#endif /* __EXAMPLE_USART_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
