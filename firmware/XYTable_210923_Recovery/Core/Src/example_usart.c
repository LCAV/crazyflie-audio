/**
 ******************************************************************************
 * @file       nucleo_xnucleo_usart.c
 * @date       01/10/2014 12:00:00
 * @brief      This file provides the functions to send commands to the L6470
 *             via Nucleo USART 
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

#include "example_usart.h"
#include "xnucleoihm02a1.h"

/**
 * @addtogroup MicrosteppingMotor_Example
 * @{
 */

/**
 * @addtogroup   ExampleUsart
 * @{
 */

/**
 * @addtogroup ExampleUsartPrivateTypes
 * @{
 */

/**
 * @brief The states while USART text decoding
 */
typedef enum {
	DECODE_MOTOR,       //!< Motor name decoding
	DECODE_COMMAND,     //!< L6470 Application Commad decoding
	DECODE_1st_PARAM,   //!< L6470 Application Commad 1st parameter decoding
	DECODE_2nd_PARAM,   //!< L6470 Application Commad 2nd parameter decoding
	DECODE_3rd_PARAM,   //!< L6470 Application Commad 3rd parameter decoding
} eL6470_UsartTextStringDecodingStatus;

/**
 * @}
 *//* End of ExampleUsartPrivateTypes */

/**
 * @addtogroup ExampleUsartPrivateMacros
 * @{
 */

#define NucleoUsartReceiveIT    HAL_UART_Receive_IT     //!< Rename the HAL function to receive an amount of data in non blocking mode.
#ifdef STM32F072xB
#define NUCLEO_BOARD_NAME       "NUCLEO-F072RB"
#define USART_STATUS_REGISTER   ISR                     //!< HAL USART status register name adapter.
#define USART_DATA_REGISTER     RDR                     //!< HAL UART data register name adapter.
#endif
#ifdef STM32F302x8
#define NUCLEO_BOARD_NAME       "NUCLEO-F302R8"
#define USART_STATUS_REGISTER   ISR                     //!< HAL USART status register name adapter.
#define USART_DATA_REGISTER     RDR                     //!< HAL UART data register name adapter.
#endif
#ifdef STM32F401xE
#define NUCLEO_BOARD_NAME       "NUCLEO-F401RE"
#define USART_STATUS_REGISTER   SR                      //!< HAL USART status register name adapter.
#define USART_DATA_REGISTER     DR                      //!< HAL UART data register name adapter.
#endif

/**
 * @}
 *//* End of ExampleUsartPrivateMacros */

/**
 * @addtogroup ExampleUsartPrivateVariables
 * @{
 */

UART_HandleTypeDef huart2; //!< The data structure for all further instances to USART2.
sL6470_DaisyChainMnemonic L6470_DaisyChainMnemonic[L6470DAISYCHAINSIZE]; //!< The mnemonic names for the L6470 in the daisy chain configuration
uint8_t UsartTextString[USARTTEXTSTRINGSIZE]; //!< To store the USART input text string.
uint8_t UsartTextString_1[USARTTEXTSTRINGSIZE]; //!< To store the USART input text string.
sL6470_TextCommandBundle L6470_TextCommandBundle[L6470DAISYCHAINSIZE]; //!< To store the splitted USART input text string into single fileds.

extern int32_t counter_X;
extern int32_t counter_Y;

/**
 * @}
 *//* End of ExampleUsartPrivateVariables */

/**
 * @defgroup   ExampleUsartPrivateFunctions
 * @{
 */

FlagStatus USART_SplitTextString(uint8_t *pTextString, sL6470_TextCommandBundle *pL6470_TextCommandBundle);
FlagStatus USART_CheckTextCommandBundle(sL6470_TextCommandBundle *pL6470_TextCommandBundle, uint8_t* pL6470_DaisyChainSpiTxStruct);
uint32_t* USART_DecodeTextString(uint8_t *pTextString, sL6470_TextCommandBundle *pL6470_TextCommandBundle, uint8_t* pL6470_DaisyChainSpiTxStruct, uint8_t* pL6470_DaisyChainSpiRxStruct);
FlagStatus CompareTwoTextString(uint8_t* TextString1, uint8_t* TextString2);
FlagStatus str2num(uint8_t* str, uint32_t* pnum);

/**
 * @}
 *//* End of ExampleUsartPrivateFunctions */

/**
 * @addtogroup ExampleUsartPrivateFunctions
 * @{
 */

/**
 * @brief Split the whole command string entered via USART into single text
 *        strings to be used
 * @retval FlagStatus  SET or RESET related to the result
 */
FlagStatus USART_SplitTextString(uint8_t *pTextString, sL6470_TextCommandBundle *pL6470_TextCommandBundle) {
	uint8_t CmdTxt_id; /* To index the text command string inside the whole text string */
	uint8_t c_id; /* To index the characters of the command string */
	uint8_t ch; /* The actual character to be checked */
	uint8_t k_id; /* To index the characters of each word of the command string */
	uint8_t k_idmax; /* To store the actual max value for k */
	eL6470_UsartTextStringDecodingStatus UsartTextStringDecodingStatus; /* The store the actual state for USART text decoding */

	for (CmdTxt_id = 0; CmdTxt_id < L6470DAISYCHAINSIZE; CmdTxt_id++) {
		*((pL6470_TextCommandBundle + CmdTxt_id)->MotorName) = '\0';
		*((pL6470_TextCommandBundle + CmdTxt_id)->CommandName) = '\0';
		*((pL6470_TextCommandBundle + CmdTxt_id)->Param[0]) = '\0';
		*((pL6470_TextCommandBundle + CmdTxt_id)->Param[1]) = '\0';
		*((pL6470_TextCommandBundle + CmdTxt_id)->Param[2]) = '\0';
	}

	c_id = 0;
	ch = *(pTextString + c_id);
	k_id = 0;
	k_idmax = 2;
	CmdTxt_id = 0;

	UsartTextStringDecodingStatus = DECODE_MOTOR;

	while (ch != '\0') {
		switch (ch) {
		case '.':
			if (UsartTextStringDecodingStatus != DECODE_3rd_PARAM) {
				UsartTextStringDecodingStatus++;
				k_id = 0;
			} else {
#ifdef NUCLEO_USE_USART
				USART_Transmit(&huart2, "Too much entered parameters.\n\r\n\r");
#endif
				return RESET;
			}
			break;
		case ',':
			CmdTxt_id++;
			if (CmdTxt_id != L6470DAISYCHAINSIZE) {
				UsartTextStringDecodingStatus = DECODE_MOTOR;
				k_id = 0;
			} else {
#ifdef NUCLEO_USE_USART
				USART_Transmit(&huart2, "Too much addressed L6470.\n\r\n\r");
#endif
				return RESET;
			}
			break;
		default:
			if (k_id != k_idmax) {
				switch (UsartTextStringDecodingStatus) {
				case DECODE_MOTOR:
					k_idmax = 2;
					*(((pL6470_TextCommandBundle + CmdTxt_id)->MotorName) + k_id) = ch;
					*(((pL6470_TextCommandBundle + CmdTxt_id)->MotorName) + (k_id + 1)) = '\0';
					break;
				case DECODE_COMMAND:
					k_idmax = 11;
					*(((pL6470_TextCommandBundle + CmdTxt_id)->CommandName) + k_id) = ch;
					*(((pL6470_TextCommandBundle + CmdTxt_id)->CommandName) + (k_id + 1)) = '\0';
					break;
				case DECODE_1st_PARAM:
					k_idmax = 10;
					*(((pL6470_TextCommandBundle + CmdTxt_id)->Param[0]) + k_id) = ch;
					*(((pL6470_TextCommandBundle + CmdTxt_id)->Param[0]) + (k_id + 1)) = '\0';
					break;
				case DECODE_2nd_PARAM:
					k_idmax = 7;
					*(((pL6470_TextCommandBundle + CmdTxt_id)->Param[1]) + k_id) = ch;
					*(((pL6470_TextCommandBundle + CmdTxt_id)->Param[1]) + (k_id + 1)) = '\0';
					break;
				case DECODE_3rd_PARAM:
					k_idmax = 7;
					*(((pL6470_TextCommandBundle + CmdTxt_id)->Param[2]) + k_id) = ch;
					*(((pL6470_TextCommandBundle + CmdTxt_id)->Param[2]) + (k_id + 1)) = '\0';
					break;
				default:
					break;
				}
				k_id++;
			} else {
#ifdef NUCLEO_USE_USART
				USART_Transmit(&huart2, "Too much entered character.\n\r\n\r");
#endif
				return RESET;
			}
			break;
		}

		c_id++;
		ch = *(pTextString + c_id);
	}
	return SET;
}

/**
 * @brief Check the entered command for L6470
 * @retval FlagStatus  SET or RESET related to the result
 */
FlagStatus USART_CheckTextCommandBundle(sL6470_TextCommandBundle *pL6470_TextCommandBundle, uint8_t* pL6470_DaisyChainSpiTxStruct) {
	uint8_t PkgId;
	uint8_t spibyte;
	uint8_t i;
	uint8_t CmdTxt_id; /* To index the string command */
	uint8_t MotorMnemonic[4] = "";
	uint8_t* pMotorMnemonic = MotorMnemonic;
	uint8_t L6470_Id;
	eL6470_AppCmdId_t L6470_AppCmdId;
	uint8_t nr_parameters; /* The max number of parameters related to the actual L6470 Application Command */
	eL6470_RegId_t L6470_RegId;
	uint8_t PARAMLengthBytes; /* The number of bytes related to the numeric value for the addressed register */
	uint32_t NumericValue; /* The numeric value to be used into the application command */
	eL6470_DirId_t L6470_DirId;
	eL6470_ActId_t L6470_ActId;

	/* Reset the structure used to send the command to the L6470 Daisy Chain through the SPI */
	i = 0;
	for (spibyte = 0; spibyte < L6470MAXSPICMDBYTESIZE; spibyte++)
		for (PkgId = 0; PkgId < L6470DAISYCHAINSIZE; PkgId++)
			*(pL6470_DaisyChainSpiTxStruct + (i++)) = 0x00;

	/* Reset the object pointed by mMnemonic */
	*pMotorMnemonic = '\0';

	CmdTxt_id = 0;
	do {

		/* Checks which L6470 in the daisy chain has to be addressed ************** */
		i = 0;
		while (!CompareTwoTextString((uint8_t*) (pL6470_TextCommandBundle + CmdTxt_id)->MotorName, (uint8_t*) L6470_DaisyChainMnemonic[i].MotorIdMnemonic)) {
			i++;
			if (i == L6470DAISYCHAINSIZE) {
#ifdef NUCLEO_USE_USART
				USART_Transmit(&huart2, "It is not possible to identify the L6470. \n\r\n\r");
#endif
				return RESET;
			}
		}

		/* The L6470 to address is known */
		L6470_Id = L6470_ID(i);

		/* Check for different addressed L6470 */
		if (!CompareTwoTextString((uint8_t*) L6470_DaisyChainMnemonic[i].MotorIdMnemonic, pMotorMnemonic)) {
			pMotorMnemonic = (uint8_t*) L6470_DaisyChainMnemonic[i].MotorIdMnemonic;
			/* ************************************************************************ */

			/* Checks which Application Command has to be performed ******************* */
			L6470_AppCmdId = (eL6470_AppCmdId_t) 0;
			while (!CompareTwoTextString((uint8_t*) (pL6470_TextCommandBundle + CmdTxt_id)->CommandName, (uint8_t*) L6470_ApplicationCommand[L6470_AppCmdId].Mnemonic)) {
				L6470_AppCmdId++;
				if (L6470_AppCmdId == L6470APPCMDIDSIZE) {
#ifdef NUCLEO_USE_USART
					USART_Transmit(&huart2, "It is not possible to recognize any L6470 application command.\n\r\n\r");
#endif
					return RESET;
				}
			}
			/* The application Command to perform is known */
			/* The max number of parameters has to be the following */
			nr_parameters = L6470_ApplicationCommand[L6470_AppCmdId].NrOfParameters;

			/* Check for a right number of entered command parameters */
			i = 0;
			while (i < nr_parameters) {
				if ((*((pL6470_TextCommandBundle + CmdTxt_id)->Param[i])) == '\0') {
#ifdef NUCLEO_USE_USART
					USART_Transmit(&huart2, "The number of parameters related to the application command is not right.\n\r\n\r");
#endif
					return RESET;
				} else {
					i++;
				}
			}
			if (i < 3) /* Max possible parameters */
			{
				if ((*((pL6470_TextCommandBundle + CmdTxt_id)->Param[i])) != '\0') {
#ifdef NUCLEO_USE_USART
					USART_Transmit(&huart2, "The number of parameters related to the application command is not right.\n\r\n\r");
#endif
					return RESET;
				}
			}
			/* ************************************************************************ */

			/* Checks the application command parameter ******************************* */
			switch (L6470_AppCmdId) {
			case L6470_NOP_ID:
				*(pL6470_DaisyChainSpiTxStruct + ((0 * L6470DAISYCHAINSIZE) + L6470_Id)) = 0x00;
				break;
			case L6470_SETPARAM_ID:
				/* Check for possible Register to be addresses */
				L6470_RegId = (eL6470_RegId_t) 0;
				while (!CompareTwoTextString((uint8_t*) (pL6470_TextCommandBundle + CmdTxt_id)->Param[0], (uint8_t*) L6470_Register[L6470_RegId].Name)) {
					L6470_RegId++;
					if (L6470_RegId == L6470REGIDSIZE) {
#ifdef NUCLEO_USE_USART
						USART_Transmit(&huart2, "It is not possible to recognize any L6470 register name.\n\r\n\r");
#endif
						return RESET;
					}
				}
				/* The 1st application command parameter (as PARAM) is known ************ */
				*(pL6470_DaisyChainSpiTxStruct + ((0 * L6470DAISYCHAINSIZE) + L6470_Id)) = 0x00 | L6470_Register[L6470_RegId].Address;

				/* The length, in byte, of this register (PARAM) is... */
				PARAMLengthBytes = L6470_Register[L6470_RegId].LengthByte;

				/* Checks the numeric parameter that has been entered (as 2nd parameter) */
				if (!str2num((uint8_t*) (pL6470_TextCommandBundle + CmdTxt_id)->Param[1], &NumericValue)) {
#ifdef NUCLEO_USE_USART
					USART_Transmit(&huart2, "It is not possible to recognize any numeric value.\n\r\n\r");
#endif
					return RESET;
				}
				/* The 2nd application command parameter (as a numeric value) is known */

				/* Build the others bytes to transmit (VALUE) */
				for (spibyte = 1; spibyte < (PARAMLengthBytes + 1); spibyte++) {
					*(pL6470_DaisyChainSpiTxStruct + ((spibyte * L6470DAISYCHAINSIZE) + L6470_Id)) = (uint8_t) (NumericValue >> (8 * (PARAMLengthBytes - spibyte)));
				}
				break;
			case L6470_GETPARAM_ID:
				/* Check for possible Register to be addresses */
				L6470_RegId = (eL6470_RegId_t) 0;
				while (!CompareTwoTextString((uint8_t*) (pL6470_TextCommandBundle + CmdTxt_id)->Param[0], (uint8_t*) L6470_Register[L6470_RegId].Name)) {
					L6470_RegId++;
					if (L6470_RegId == L6470REGIDSIZE) {
#ifdef NUCLEO_USE_USART
						USART_Transmit(&huart2, "It is not possible to recognize any L6470 register name.\n\r\n\r");
#endif
						return RESET;
					}
				}
				/* The 1st application command parameter (as PARAM) is known ************ */
				*(pL6470_DaisyChainSpiTxStruct + ((0 * L6470DAISYCHAINSIZE) + L6470_Id)) = 0x20 | L6470_Register[L6470_RegId].Address;

				/* The length, in byte, of this register (PARAM) is... */
				PARAMLengthBytes = L6470_Register[L6470_RegId].LengthByte;

				/* Build the others bytes to transmit (VALUE) */
				for (spibyte = 1; spibyte < (PARAMLengthBytes + 1); spibyte++) {
					*(pL6470_DaisyChainSpiTxStruct + ((spibyte * L6470DAISYCHAINSIZE) + L6470_Id)) = 0x00;
				}
				break;
			case L6470_RUN_ID:
				/* Checks the Motor Direction that has been entered (as 1st parameter) */
				L6470_DirId = (eL6470_DirId_t) 0;
				while (!CompareTwoTextString((uint8_t*) (pL6470_TextCommandBundle + CmdTxt_id)->Param[0], (uint8_t*) L6470_Direction[L6470_DirId].Mnemonic)) {
					L6470_DirId++;
					if (L6470_DirId == L6470DIRIDSIZE) {
#ifdef NUCLEO_USE_USART
						USART_Transmit(&huart2, "It is not possible to recognize any direction.\n\r\n\r");
#endif
						return RESET;
					}
				}
				/* The 1st application command parameter (as DIR) is known ************** */
				*(pL6470_DaisyChainSpiTxStruct + ((0 * L6470DAISYCHAINSIZE) + L6470_Id)) = 0x50 | L6470_DirId;

				/* Checks the numeric parameter that has been entered (as 2nd parameter) */
				if (!str2num((uint8_t*) (pL6470_TextCommandBundle + CmdTxt_id)->Param[1], &NumericValue)) {
#ifdef NUCLEO_USE_USART
					USART_Transmit(&huart2, "It is not possible to recognize any numeric value.\n\r\n\r");
#endif
					return RESET;
				}
				/* The 2nd application command parameter (as a numeric value) is known */

				/* Build the others bytes to transmit (SPD) */
				for (spibyte = 1; spibyte < (3 + 1); spibyte++) {
					*(pL6470_DaisyChainSpiTxStruct + ((spibyte * L6470DAISYCHAINSIZE) + L6470_Id)) = (uint8_t) (NumericValue >> (8 * (3 - spibyte)));
				}
				break;
			case L6470_STEPCLOCK_ID:
				/* Checks the Motor Direction that has been entered */
				L6470_DirId = (eL6470_DirId_t) 0;
				while (!CompareTwoTextString((uint8_t*) (pL6470_TextCommandBundle + CmdTxt_id)->Param[0], (uint8_t*) L6470_Direction[L6470_DirId].Mnemonic)) {
					L6470_DirId++;
					if (L6470_DirId == L6470DIRIDSIZE) {
#ifdef NUCLEO_USE_USART
						USART_Transmit(&huart2, "It is not possible to recognize any direction.\n\r\n\r");
#endif
						return RESET;
					}
				}
				/* The application command parameter (as DIR) is known ************** */
				*(pL6470_DaisyChainSpiTxStruct + ((0 * L6470DAISYCHAINSIZE) + L6470_Id)) = 0x58 | L6470_DirId;
				break;
			case L6470_MOVE_ID:
				/* Checks the Motor Direction that has been entered (as 1st parameter) */
				L6470_DirId = (eL6470_DirId_t) 0;
				while (!CompareTwoTextString((uint8_t*) (pL6470_TextCommandBundle + CmdTxt_id)->Param[0], (uint8_t*) L6470_Direction[L6470_DirId].Mnemonic)) {
					L6470_DirId++;
					if (L6470_DirId == L6470DIRIDSIZE) {
#ifdef NUCLEO_USE_USART
						USART_Transmit(&huart2, "It is not possible to recognize any direction.\n\r\n\r");
#endif
						return RESET;
					}
				}
				/* The 1st application command parameter (as DIR) is known ************** */
				*(pL6470_DaisyChainSpiTxStruct + ((0 * L6470DAISYCHAINSIZE) + L6470_Id)) = 0x40 | L6470_DirId;

				/* Checks the numeric parameter that has been entered (as 2nd parameter) */
				if (!str2num((uint8_t*) (pL6470_TextCommandBundle + CmdTxt_id)->Param[1], &NumericValue)) {
#ifdef NUCLEO_USE_USART
					USART_Transmit(&huart2, "It is not possible to recognize any numeric value.\n\r\n\r");
#endif
					return RESET;
				}
				/* The 2nd application command parameter (as a numeric value) is known */

				/* Build the others bytes to transmit (N_STEP) */
				for (spibyte = 1; spibyte < (3 + 1); spibyte++) {
					*(pL6470_DaisyChainSpiTxStruct + ((spibyte * L6470DAISYCHAINSIZE) + L6470_Id)) = (uint8_t) (NumericValue >> (8 * (3 - spibyte)));
				}
				break;
			case L6470_GOTO_ID:
				/* Build the 1st byte to transmit */
				*(pL6470_DaisyChainSpiTxStruct + ((0 * L6470DAISYCHAINSIZE) + L6470_Id)) = 0x60;

				/* Checks the numeric parameter that has been entered */
				if (!str2num((uint8_t*) (pL6470_TextCommandBundle + CmdTxt_id)->Param[0], &NumericValue)) {
#ifdef NUCLEO_USE_USART
					USART_Transmit(&huart2, "It is not possible to recognize any numeric value.\n\r\n\r");
#endif
					return RESET;
				}
				/* The application command parameter (as a numeric value) is known */

				/* Build the others bytes to transmit (ABS_POS) */
				for (spibyte = 1; spibyte < (3 + 1); spibyte++) {
					*(pL6470_DaisyChainSpiTxStruct + ((spibyte * L6470DAISYCHAINSIZE) + L6470_Id)) = (uint8_t) (NumericValue >> (8 * (3 - spibyte)));
				}
				break;
			case L6470_GOTODIR_ID:
				/* Checks the Motor Direction that has been entered (as 1st parameter) */
				L6470_DirId = (eL6470_DirId_t) 0;
				while (!CompareTwoTextString((uint8_t*) (pL6470_TextCommandBundle + CmdTxt_id)->Param[0], (uint8_t*) L6470_Direction[L6470_DirId].Mnemonic)) {
					L6470_DirId++;
					if (L6470_DirId == L6470DIRIDSIZE) {
#ifdef NUCLEO_USE_USART
						USART_Transmit(&huart2, "It is not possible to recognize any direction.\n\r\n\r");
#endif
						return RESET;
					}
				}
				/* The 1st application command parameter (as DIR) is known ************** */
				/* Build the 1st byte to transmit */
				*(pL6470_DaisyChainSpiTxStruct + ((0 * L6470DAISYCHAINSIZE) + L6470_Id)) = 0x68 | L6470_DirId;

				/* Checks the numeric parameter that has been entered (as 2nd parameter) */
				if (!str2num((uint8_t*) (pL6470_TextCommandBundle + CmdTxt_id)->Param[1], &NumericValue)) {
#ifdef NUCLEO_USE_USART
					USART_Transmit(&huart2, "It is not possible to recognize any numeric value.\n\r\n\r");
#endif
					return RESET;
				}
				/* The 2nd application command parameter (as a numeric value) is known */

				/* Build the others bytes to transmit (ABS_POS) */
				for (spibyte = 1; spibyte < (3 + 1); spibyte++) {
					*(pL6470_DaisyChainSpiTxStruct + ((spibyte * L6470DAISYCHAINSIZE) + L6470_Id)) = (uint8_t) (NumericValue >> (8 * (3 - spibyte)));
				}
				break;
			case L6470_GOUNTIL_ID:
				/* Checks the Action about ABS_POS register that has been entered (as 1st parameter) */
				L6470_ActId = (eL6470_ActId_t) 0;
				while (!CompareTwoTextString((uint8_t*) (pL6470_TextCommandBundle + CmdTxt_id)->Param[0], (uint8_t*) L6470_ACT[L6470_ActId].Mnemonic)) {
					L6470_ActId++;
					if (L6470_ActId == L6470ACTIDSIZE) {
#ifdef NUCLEO_USE_USART
						USART_Transmit(&huart2, "It is not possible to recognize any action.\n\r\n\r");
#endif
						return RESET;
					}
				}
				/* The 1st application command parameter (as ACT) is known **************** */
				/* Build the 1st byte to transmit */
				*(pL6470_DaisyChainSpiTxStruct + ((0 * L6470DAISYCHAINSIZE) + L6470_Id)) = 0x82 | (L6470_ActId << 3);

				/* Checks the Motor Direction that has been entered (as 2nd parameter) */
				L6470_DirId = (eL6470_DirId_t) 0;
				while (!CompareTwoTextString((uint8_t*) (pL6470_TextCommandBundle + CmdTxt_id)->Param[1], (uint8_t*) L6470_Direction[L6470_DirId].Mnemonic)) {
					L6470_DirId++;
					if (L6470_DirId == L6470DIRIDSIZE) {
#ifdef NUCLEO_USE_USART
						USART_Transmit(&huart2, "It is not possible to recognize any direction.\n\r\n\r");
#endif
						return RESET;
					}
				}
				/* The 2nd application command parameter (as DIR) is known ************** */
				/* Build the 1st byte to transmit */
				*(pL6470_DaisyChainSpiTxStruct + ((0 * L6470DAISYCHAINSIZE) + L6470_Id)) |= L6470_DirId;

				/* Checks the numeric parameter that has been entered (as 3rd parameter) */
				if (!str2num((uint8_t*) (pL6470_TextCommandBundle + CmdTxt_id)->Param[2], &NumericValue)) {
#ifdef NUCLEO_USE_USART
					USART_Transmit(&huart2, "It is not possible to recognize any numeric value.\n\r\n\r");
#endif
					return RESET;
				}
				/* The 3rd application command parameter (as a numeric value) is known */

				/* Build the others bytes to transmit (SPD) */
				for (spibyte = 1; spibyte < (3 + 1); spibyte++) {
					*(pL6470_DaisyChainSpiTxStruct + ((spibyte * L6470DAISYCHAINSIZE) + L6470_Id)) = (uint8_t) (NumericValue >> (8 * (3 - spibyte)));
				}
				break;
			case L6470_RELEASESW_ID:
				/* Checks the Action about ABS_POS register that has been entered (as 1st parameter) */
				L6470_ActId = (eL6470_ActId_t) 0;
				while (!CompareTwoTextString((uint8_t*) (pL6470_TextCommandBundle + CmdTxt_id)->Param[0], (uint8_t*) L6470_ACT[L6470_ActId].Mnemonic)) {
					L6470_ActId++;
					if (L6470_ActId == L6470ACTIDSIZE) {
#ifdef NUCLEO_USE_USART
						USART_Transmit(&huart2, "It is not possible to recognize any action.\n\r\n\r");
#endif
						return RESET;
					}
				}
				/* The 1st application command parameter (as ACT) is known **************** */
				/* Build the 1st byte to transmit */
				*(pL6470_DaisyChainSpiTxStruct + ((0 * L6470DAISYCHAINSIZE) + L6470_Id)) = 0x92 | (L6470_ActId << 3);

				/* Checks the Motor Direction that has been entered (as 2nd parameter) */
				L6470_DirId = (eL6470_DirId_t) 0;
				while (!CompareTwoTextString((uint8_t*) (pL6470_TextCommandBundle + CmdTxt_id)->Param[1], (uint8_t*) L6470_Direction[L6470_DirId].Mnemonic)) {
					L6470_DirId++;
					if (L6470_DirId == L6470DIRIDSIZE) {
#ifdef NUCLEO_USE_USART
						USART_Transmit(&huart2, "It is not possible to recognize any direction.\n\r\n\r");
#endif
						return RESET;
					}
				}
				/* The 2nd application command parameter (as DIR) is known ************** */
				/* Build the 1st byte to transmit */
				*(pL6470_DaisyChainSpiTxStruct + ((0 * L6470DAISYCHAINSIZE) + L6470_Id)) |= L6470_DirId;
				break;
			case L6470_GOHOME_ID:
				/* Build the 1st byte to transmit */
				*(pL6470_DaisyChainSpiTxStruct + ((0 * L6470DAISYCHAINSIZE) + L6470_Id)) = 0x70;
				break;
			case L6470_GOMARK_ID:
				/* Build the 1st byte to transmit */
				*(pL6470_DaisyChainSpiTxStruct + ((0 * L6470DAISYCHAINSIZE) + L6470_Id)) = 0x78;
				break;
			case L6470_RESETPOS_ID:
				/* Build the 1st byte to transmit */
				*(pL6470_DaisyChainSpiTxStruct + ((0 * L6470DAISYCHAINSIZE) + L6470_Id)) = 0xD8;
				break;
			case L6470_RESETDEVICE_ID:
				/* Build the 1st byte to transmit */
				*(pL6470_DaisyChainSpiTxStruct + ((0 * L6470DAISYCHAINSIZE) + L6470_Id)) = 0xC0;
				break;
			case L6470_SOFTSTOP_ID:
				/* Build the 1st byte to transmit */
				*(pL6470_DaisyChainSpiTxStruct + ((0 * L6470DAISYCHAINSIZE) + L6470_Id)) = 0xB0;
				break;
			case L6470_HARDSTOP_ID:
				/* Build the 1st byte to transmit */
				*(pL6470_DaisyChainSpiTxStruct + ((0 * L6470DAISYCHAINSIZE) + L6470_Id)) = 0xB8;
				break;
			case L6470_SOFTHIZ_ID:
				/* Build the 1st byte to transmit */
				*(pL6470_DaisyChainSpiTxStruct + ((0 * L6470DAISYCHAINSIZE) + L6470_Id)) = 0xA0;
				break;
			case L6470_HARDHIZ_ID:
				/* Build the 1st byte to transmit */
				*(pL6470_DaisyChainSpiTxStruct + ((0 * L6470DAISYCHAINSIZE) + L6470_Id)) = 0xA8;
				break;
			case L6470_GETSTATUS_ID:
				/* Build the 1st byte to transmit */
				*(pL6470_DaisyChainSpiTxStruct + ((0 * L6470DAISYCHAINSIZE) + L6470_Id)) = 0xD0;
				/* Build the others bytes to transmit (SPD) */
				for (spibyte = 1; spibyte < 2 + 1; spibyte++) {
					*(pL6470_DaisyChainSpiTxStruct + ((spibyte * L6470DAISYCHAINSIZE) + L6470_Id)) = 0x00;
				}
				break;
			}
		} else {
#ifdef NUCLEO_USE_USART
			USART_Transmit(&huart2, "Both entered commands are addressing the same L6470.\n\r\n\r");
#endif
			return RESET;
		}
		CmdTxt_id++;
	} while ((CmdTxt_id < L6470DAISYCHAINSIZE)
			&& (((*((pL6470_TextCommandBundle + CmdTxt_id)->MotorName)) != '\0') || ((*((pL6470_TextCommandBundle + CmdTxt_id)->CommandName)) != '\0')
					|| ((*((pL6470_TextCommandBundle + CmdTxt_id)->Param[0])) != '\0') || ((*((pL6470_TextCommandBundle + CmdTxt_id)->Param[1])) != '\0')
					|| ((*((pL6470_TextCommandBundle + CmdTxt_id)->Param[2])) != '\0')));

	return SET;
}

/**
 * @brief Decode the command string entered via USART
 * @retval None
 * @note It use two function, one to split the whole entered command string and
 *       another one to check for the application command
 */
uint32_t* USART_DecodeTextString(uint8_t *pTextString, sL6470_TextCommandBundle *pL6470_TextCommandBundle, uint8_t* pL6470_DaisyChainSpiTxStruct, uint8_t* pL6470_DaisyChainSpiRxStruct) {
	static uint32_t ReceivedValue[L6470DAISYCHAINSIZE];
	uint8_t L6470_Id;
	eL6470_RegId_t L6470_RegId;
	uint8_t PARAMLengthBytes; /* The number of bytes related to the numeric value for the addressed register */
	eHexFormat HexFormat;

	if (USART_SplitTextString(pTextString, pL6470_TextCommandBundle) && USART_CheckTextCommandBundle(pL6470_TextCommandBundle, pL6470_DaisyChainSpiTxStruct)) {
#ifdef NUCLEO_USE_USART
		char string[60] = "";
		/* counter_X /(40*128) ; counter_Y / (100*128) */
		sprintf(string, "Wait end of movement..., current position: x = %d [mm*10], y = %d[mm*10]", (int)((float)counter_X * 10 / (100.0*128)), (int)((float)counter_Y * 10 / (100.0*128)));

		USART_Transmit(&huart2, string);
#endif
		L6470_DaisyChainCommand(pL6470_DaisyChainSpiTxStruct, pL6470_DaisyChainSpiRxStruct);

		/* Check if a value has to be returned */
		for (L6470_Id = 0; L6470_Id < L6470DAISYCHAINSIZE; L6470_Id++) {
			/* Reset the value before fill with the new value */
			ReceivedValue[L6470_Id] = 0x00;

			if ((*(pL6470_DaisyChainSpiTxStruct + ((0 * L6470DAISYCHAINSIZE) + L6470_Id)) & 0xE0) == 0x20) // The AppCmd has been "GetParam"
					{
				/* The length, in byte, of this register (PARAM) is... */
				/* Check for the Register address */
				L6470_RegId = (eL6470_RegId_t) 0;
				while ((*(pL6470_DaisyChainSpiTxStruct + ((0 * L6470DAISYCHAINSIZE) + L6470_Id)) & 0x1F) != (L6470_Register[L6470_RegId].Address))
					L6470_RegId++;
				PARAMLengthBytes = L6470_Register[L6470_RegId].LengthByte;
				switch (PARAMLengthBytes) {
				case 1:
					HexFormat = BYTE_F;
					break;
				case 2:
					HexFormat = WORD_F;
					break;
				default:
					HexFormat = DOUBLEWORD_F;
					break;
				}

				ReceivedValue[L6470_Id] = L6470_ExtractReturnedData(L6470_Id, pL6470_DaisyChainSpiRxStruct, PARAMLengthBytes);

#ifdef NUCLEO_USE_USART
				USART_Transmit(&huart2, (uint8_t*) (L6470_DaisyChainMnemonic[L6470_Id].L6470IdMnemonic));
				USART_Transmit(&huart2, " ");
				USART_Transmit(&huart2, (uint8_t*) (L6470_Register[L6470_RegId].Name));
				USART_Transmit(&huart2, " value is 0x");

				USART_Transmit(&huart2, num2hex(ReceivedValue[L6470_Id], HexFormat));

				USART_Transmit(&huart2, "\n\r");
#endif        
			}

			if (*(pL6470_DaisyChainSpiTxStruct + ((0 * L6470DAISYCHAINSIZE) + L6470_Id)) == 0xD0) // The AppCmd has been "GetStatus"
					{
				ReceivedValue[L6470_Id] = L6470_ExtractReturnedData(L6470_Id, pL6470_DaisyChainSpiRxStruct, 2);

#ifdef NUCLEO_USE_USART
				USART_Transmit(&huart2, (uint8_t*) (L6470_DaisyChainMnemonic[L6470_Id].L6470IdMnemonic));
				USART_Transmit(&huart2, " STATUS value is 0x");

				USART_Transmit(&huart2, num2hex(ReceivedValue[L6470_Id], WORD_F));

				USART_Transmit(&huart2, "\n\r");
#endif        
			}
		}
#ifdef NUCLEO_USE_USART
		USART_Transmit(&huart2, "\n\r");
#endif    
	} else {
#ifdef NUCLEO_USE_USART
		USART_Transmit(&huart2, "Please, enter a new command string!\n\r\n\r");
#endif
	}

	return ReceivedValue;
}

/**
 * @brief  This function compares two text strings.
 * @param  TextString1 The pointer to the 1st text string to compare.
 * @param  TextString2 The pointer to the 2nd text string to compare.
 * @retval FlagStatus    SET or RESET related the comparison.
 */
FlagStatus CompareTwoTextString(uint8_t* TextString1, uint8_t* TextString2) {
	uint8_t i; /* to index the two text strings to be compared */
	uint8_t c1, c2; /* to store the two characters to be compared */

	i = 0;

	do {
		c1 = *(TextString1 + i);
		c2 = *(TextString2 + i);

		if (c1 != c2) {
			return RESET;
		}
		i++;
	} while ((c1 != '\0') && (c2 != '\0'));

	return SET;
}

/**
 * @brief  This function converts a text string into a number.
 * @param  str       The pointer to the text string to convert.
 * @param  pnum      The pointer to the numerical variable.
 * @retval FlagStatus  SET or RESET related to the conversion.
 */
FlagStatus str2num(uint8_t* str, uint32_t* pnum) {
	uint8_t TxtStr_digit, digit;
	uint32_t tenpwr;
	uint32_t number;

	digit = 0;

	while (*(str + digit) != '\0') {
		if (((*(str + digit) >= '0') && (*(str + digit) <= '9'))) {
			digit++;
		} else {
			*pnum = 0;
			return RESET;
		}
	}

	tenpwr = 1;
	number = 0;

	do {
		TxtStr_digit = (*(str + (--digit)));
		number += ((TxtStr_digit - '0') * tenpwr);
		tenpwr *= 10;
	} while (digit);

	*pnum = number;
	return SET;
}

/**
 * @}
 *//* End of ExampleUsartPrivateFunctions */

/**
 * @addtogroup   ExampleUsartExportedFunctions
 * @{
 */

/**
 * @brief  Transmit the initial message to the PC which is connected to the
 *         NUCLEO board via UART.
 */
void USART_TxWelcomeMessage(void) {
	/* Send information to PC via USART */
	USART_Transmit(&huart2, "\n\r");
	USART_Transmit(&huart2, " X-NUCLEO-IHM02A1\n\r");
	USART_Transmit(&huart2, " -------------------------------------------\n\r");
	USART_Transmit(&huart2, " Dual L6470 Expansion Board for STM32 NUCLEO\n\r");
	USART_Transmit(&huart2, " Stacked on ");
	USART_Transmit(&huart2, NUCLEO_BOARD_NAME);
	USART_Transmit(&huart2, " \n\r");
	USART_Transmit(&huart2, " X-CUBE-SPN2 v1.0.0\n\r");
	USART_Transmit(&huart2, " STMicroelectronics, 2015, Edited by Adrien Hoffet, 12.2017\n\r");
	USART_Transmit(&huart2, " Xmotor: Q,W,E/A,S,D (FORWARD/BACKWARD), (50mm, 10mm, 0.5mm)\n\r");
	USART_Transmit(&huart2, " Ymotor: P,O,I/L,K,J (FORWARD/BACKWARD), (360, 90, 27deg)\n\r");
	USART_Transmit(&huart2, " ZERO: Z (SOFTSTOP, ZERO COUNTER)\n\r\n\r");
}

/**
 * @brief  Send a text string via USART.
 * @param  huart       pointer to a UART_HandleTypeDef structure that contains
 *                     the configuration information for the specified UART module.
 * @param  TextString  The text string to be sent.
 * @note It use the HAL_UART_Transmit function.
 */
void USART_Transmit(UART_HandleTypeDef* huart, uint8_t* TextString) {
	uint8_t TextStringLength;

	/* Calculate the length of the text string to be sent */
	TextStringLength = 0;
	while (TextString[TextStringLength++] != '\0')
		;
	TextStringLength--;

	/* Use the HAL function to send the text string via USART */
	HAL_UART_Transmit(huart, TextString, TextStringLength, 10);
}

/**
 * @brief  Check if any Application Command for L6470 has been entered by USART
 *         so to proceed to decode and perform the command.
 */
void USART_CheckAppCmd(void) {
	/* Checks the UART2 is in idle state */
	if (huart2.gState == HAL_UART_STATE_READY) {
		/* Checks one character has been at least entered */
		if (UsartTextString_1[0] != '\0') {
			/* Decode the entered command string */

#if DECODE_KEYBOARD_REMOTE
			switch (UsartTextString_1[0]) {
			case 'Q':
			case 'q':
				/*
				 20*128
				 * */
				UsartTextString[0] = 'M';
				UsartTextString[1] = '1';
				UsartTextString[2] = '.';
				UsartTextString[3] = 'M';
				UsartTextString[4] = 'O';
				UsartTextString[5] = 'V';
				UsartTextString[6] = 'E';
				UsartTextString[7] = '.';
				UsartTextString[8] = 'F';
				UsartTextString[9] = 'W';
				UsartTextString[10] = 'D';
				UsartTextString[11] = '.';
				UsartTextString[12] = '6';
				UsartTextString[13] = '4';
				UsartTextString[14] = '0';
				UsartTextString[15] = '0';
				UsartTextString[16] = '\0';
				UsartTextString[17] = '\0';
				UsartTextString[18] = '\0';
				UsartTextString[19] = '\0';
				UsartTextString[20] = '\0';
				counter_X += 6400;
				break;
			case 'A':
			case 'a':
				/*
				 * -20*128
				 */
				UsartTextString[0] = 'M';
				UsartTextString[1] = '1';
				UsartTextString[2] = '.';
				UsartTextString[3] = 'M';
				UsartTextString[4] = 'O';
				UsartTextString[5] = 'V';
				UsartTextString[6] = 'E';
				UsartTextString[7] = '.';
				UsartTextString[8] = 'R';
				UsartTextString[9] = 'E';
				UsartTextString[10] = 'V';
				UsartTextString[11] = '.';
				UsartTextString[12] = '6';
				UsartTextString[13] = '4';
				UsartTextString[14] = '0';
				UsartTextString[15] = '0';
				UsartTextString[16] = '\0';
				UsartTextString[17] = '\0';
				UsartTextString[18] = '\0';
				UsartTextString[19] = '\0';
				UsartTextString[20] = '\0';
				counter_X -= 6400;

				break;
			case 'W':
			case 'w':
				/*
				 * 200*128
				 */
				UsartTextString[0] = 'M';
				UsartTextString[1] = '1';
				UsartTextString[2] = '.';
				UsartTextString[3] = 'M';
				UsartTextString[4] = 'O';
				UsartTextString[5] = 'V';
				UsartTextString[6] = 'E';
				UsartTextString[7] = '.';
				UsartTextString[8] = 'F';
				UsartTextString[9] = 'W';
				UsartTextString[10] = 'D';
				UsartTextString[11] = '.';
				UsartTextString[12] = '1';
				UsartTextString[13] = '2';
				UsartTextString[14] = '8';
				UsartTextString[15] = '0';
				UsartTextString[16] = '0';
				UsartTextString[17] = '0';
				UsartTextString[18] = '\0';
				UsartTextString[19] = '\0';
				UsartTextString[20] = '\0';
				counter_X += 64000;

				break;
			case 'S':
			case 's':
				/*
				 * -200*128
				 */
				UsartTextString[0] = 'M';
				UsartTextString[1] = '1';
				UsartTextString[2] = '.';
				UsartTextString[3] = 'M';
				UsartTextString[4] = 'O';
				UsartTextString[5] = 'V';
				UsartTextString[6] = 'E';
				UsartTextString[7] = '.';
				UsartTextString[8] = 'R';
				UsartTextString[9] = 'E';
				UsartTextString[10] = 'V';
				UsartTextString[11] = '.';
				UsartTextString[12] = '1';
				UsartTextString[13] = '2';
				UsartTextString[14] = '8';
				UsartTextString[15] = '0';
				UsartTextString[16] = '0';
				UsartTextString[17] = '0';
				UsartTextString[18] = '\0';
				UsartTextString[19] = '\0';
				UsartTextString[20] = '\0';
				counter_X -= 64000;

				break;
			case 'E':
			case 'e':
				/*
				 * 2000*128
				 */
				UsartTextString[0] = 'M';
				UsartTextString[1] = '1';
				UsartTextString[2] = '.';
				UsartTextString[3] = 'M';
				UsartTextString[4] = 'O';
				UsartTextString[5] = 'V';
				UsartTextString[6] = 'E';
				UsartTextString[7] = '.';
				UsartTextString[8] = 'F';
				UsartTextString[9] = 'W';
				UsartTextString[10] = 'D';
				UsartTextString[11] = '.';
				UsartTextString[12] = '6';
				UsartTextString[13] = '4';
				UsartTextString[14] = '0';
				UsartTextString[15] = '0';
				UsartTextString[16] = '0';
				UsartTextString[17] = '0';
				UsartTextString[18] = '\0';
				UsartTextString[19] = '\0';
				UsartTextString[20] = '\0';
				counter_X += 640000;
				break;
			case 'D':
			case 'd':
				/*
				 * -2000*128
				 */
				UsartTextString[0] = 'M';
				UsartTextString[1] = '1';
				UsartTextString[2] = '.';
				UsartTextString[3] = 'M';
				UsartTextString[4] = 'O';
				UsartTextString[5] = 'V';
				UsartTextString[6] = 'E';
				UsartTextString[7] = '.';
				UsartTextString[8] = 'R';
				UsartTextString[9] = 'E';
				UsartTextString[10] = 'V';
				UsartTextString[11] = '.';
				UsartTextString[12] = '6';
				UsartTextString[13] = '4';
				UsartTextString[14] = '0';
				UsartTextString[15] = '0';
				UsartTextString[16] = '0';
				UsartTextString[17] = '0';
				UsartTextString[18] = '\0';
				UsartTextString[19] = '\0';
				UsartTextString[20] = '\0';
				counter_X -= 640000;
				break;
			case 'P':
			case 'p':
				/*
				 * 50*128
				 */
				UsartTextString[0] = 'M'; // 30deg
				UsartTextString[1] = '0';
				UsartTextString[2] = '.';
				UsartTextString[3] = 'M';
				UsartTextString[4] = 'O';
				UsartTextString[5] = 'V';
				UsartTextString[6] = 'E';
				UsartTextString[7] = '.';
				UsartTextString[8] = 'F';
				UsartTextString[9] = 'W';
				UsartTextString[10] = 'D';
				UsartTextString[11] = '.';
				UsartTextString[12] = '1';
				UsartTextString[13] = '0';
				UsartTextString[14] = '6';
				UsartTextString[15] = '6';
				UsartTextString[16] = '6';
				UsartTextString[17] = '6';
				UsartTextString[18] = '\0';
				UsartTextString[19] = '\0';
				UsartTextString[20] = '\0';
				counter_Y += 6400;
				break;
			case 'L':
			case 'l':
				/*
				 * 50*128
				 */
				UsartTextString[0] = 'M'; // 30deg
				UsartTextString[1] = '0';
				UsartTextString[2] = '.';
				UsartTextString[3] = 'M';
				UsartTextString[4] = 'O';
				UsartTextString[5] = 'V';
				UsartTextString[6] = 'E';
				UsartTextString[7] = '.';
				UsartTextString[8] = 'R';
				UsartTextString[9] = 'E';
				UsartTextString[10] = 'V';
				UsartTextString[11] = '.';
				UsartTextString[12] = '1';
				UsartTextString[13] = '0';
				UsartTextString[14] = '6';
				UsartTextString[15] = '6';
				UsartTextString[16] = '6';
				UsartTextString[17] = '6';
				UsartTextString[18] = '\0';
				UsartTextString[19] = '\0';
				UsartTextString[20] = '\0';
				counter_Y -= 6400;
				break;
			case 'O':
			case 'o':
				/*
				 * 10*50*128
				 */
				UsartTextString[0] = 'M'; // 90deg
				UsartTextString[1] = '0';
				UsartTextString[2] = '.';
				UsartTextString[3] = 'M';
				UsartTextString[4] = 'O';
				UsartTextString[5] = 'V';
				UsartTextString[6] = 'E';
				UsartTextString[7] = '.';
				UsartTextString[8] = 'F';
				UsartTextString[9] = 'W';
				UsartTextString[10] = 'D';
				UsartTextString[11] = '.';
				UsartTextString[12] = '3';
				UsartTextString[13] = '2';
				UsartTextString[14] = '0';
				UsartTextString[15] = '0';
				UsartTextString[16] = '0';
				UsartTextString[17] = '0';
				UsartTextString[18] = '\0';
				UsartTextString[19] = '\0';
				UsartTextString[20] = '\0';
				counter_Y += 2*64000;
				break;
			case 'K':
			case 'k':
				/*
				 * 10*50*128
				 */
				UsartTextString[0] = 'M'; // 90deg
				UsartTextString[1] = '0';
				UsartTextString[2] = '.';
				UsartTextString[3] = 'M';
				UsartTextString[4] = 'O';
				UsartTextString[5] = 'V';
				UsartTextString[6] = 'E';
				UsartTextString[7] = '.';
				UsartTextString[8] = 'R';
				UsartTextString[9] = 'E';
				UsartTextString[10] = 'V';
				UsartTextString[11] = '.';
				UsartTextString[12] = '3';
				UsartTextString[13] = '2';
				UsartTextString[14] = '0';
				UsartTextString[15] = '0';
				UsartTextString[16] = '0';
				UsartTextString[17] = '0';
				UsartTextString[18] = '\0';
				UsartTextString[19] = '\0';
				UsartTextString[20] = '\0';
				counter_Y -= 2*64000;
				break;
				case 'U':
				case 'u':
					/*
					 * 100*50*128 modified
					 */
					UsartTextString[0] = 'M'; // 5deg
					UsartTextString[1] = '0';
					UsartTextString[2] = '.';
					UsartTextString[3] = 'M';
					UsartTextString[4] = 'O';
					UsartTextString[5] = 'V';
					UsartTextString[6] = 'E';
					UsartTextString[7] = '.';
					UsartTextString[8] = 'F';
					UsartTextString[9] = 'W';
					UsartTextString[10] = 'D';
					UsartTextString[11] = '.';
					UsartTextString[12] = '1';
					UsartTextString[13] = '7';
					UsartTextString[14] = '7';
					UsartTextString[15] = '7';
					UsartTextString[16] = '7';
					UsartTextString[17] = '\0';
					UsartTextString[18] = '\0';
					UsartTextString[19] = '\0';
					UsartTextString[20] = '\0';
					counter_Y += 640000;
					break;
				case 'H':
				case 'h':
					/*
					 * 100*50*128
					 */
					UsartTextString[0] = 'M'; // 5deg
					UsartTextString[1] = '0';
					UsartTextString[2] = '.';
					UsartTextString[3] = 'M';
					UsartTextString[4] = 'O';
					UsartTextString[5] = 'V';
					UsartTextString[6] = 'E';
					UsartTextString[7] = '.';
					UsartTextString[8] = 'R';
					UsartTextString[9] = 'E';
					UsartTextString[10] = 'V';
					UsartTextString[11] = '.';
					UsartTextString[12] = '1';
					UsartTextString[13] = '7';
					UsartTextString[14] = '7';
					UsartTextString[15] = '7';
					UsartTextString[16] = '7';
					UsartTextString[17] = '\0';
					UsartTextString[18] = '\0';
					UsartTextString[19] = '\0';
					UsartTextString[20] = '\0';
					counter_Y -= 640000;
					break;
			case 'I':
			case 'i':
				/*
				 * 100*50*128 modified
				 */
				UsartTextString[0] = 'M'; // 360deg
				UsartTextString[1] = '0';
				UsartTextString[2] = '.';
				UsartTextString[3] = 'M';
				UsartTextString[4] = 'O';
				UsartTextString[5] = 'V';
				UsartTextString[6] = 'E';
				UsartTextString[7] = '.';
				UsartTextString[8] = 'F';
				UsartTextString[9] = 'W';
				UsartTextString[10] = 'D';
				UsartTextString[11] = '.';
				UsartTextString[12] = '1';
				UsartTextString[13] = '2';
				UsartTextString[14] = '8';
				UsartTextString[15] = '0';
				UsartTextString[16] = '0';
				UsartTextString[17] = '0';
				UsartTextString[18] = '0';
				UsartTextString[19] = '\0';
				UsartTextString[20] = '\0';
				counter_Y += 640000;
				break;
			case 'J':
			case 'j':
				/*
				 * 100*50*128
				 */
				UsartTextString[0] = 'M'; // 360deg
				UsartTextString[1] = '0';
				UsartTextString[2] = '.';
				UsartTextString[3] = 'M';
				UsartTextString[4] = 'O';
				UsartTextString[5] = 'V';
				UsartTextString[6] = 'E';
				UsartTextString[7] = '.';
				UsartTextString[8] = 'R';
				UsartTextString[9] = 'E';
				UsartTextString[10] = 'V';
				UsartTextString[11] = '.';
				UsartTextString[12] = '1';
				UsartTextString[13] = '2';
				UsartTextString[14] = '8';
				UsartTextString[15] = '0';
				UsartTextString[16] = '0';
				UsartTextString[17] = '0';
				UsartTextString[18] = '0';
				UsartTextString[19] = '\0';
				UsartTextString[20] = '\0';
				counter_Y -= 640000;
				break;
			case 'Z':
			case 'z':
				UsartTextString[0]  = 'M';
				UsartTextString[1]  = '0';
				UsartTextString[2]  = '.';
				UsartTextString[3]  = 'S';
				UsartTextString[4]  = 'O';
				UsartTextString[5]  = 'F';
				UsartTextString[6]  = 'T';
				UsartTextString[7]  = 'S';
				UsartTextString[8]  = 'T';
				UsartTextString[9]  = 'O';
				UsartTextString[10] = 'P';
				UsartTextString[11] = ',';
				UsartTextString[12] = 'M';
				UsartTextString[13] = '1';
				UsartTextString[14] = '.';
				UsartTextString[15]  = 'S';
				UsartTextString[16]  = 'O';
				UsartTextString[17]  = 'F';
				UsartTextString[18]  = 'T';
				UsartTextString[19]  = 'S';
				UsartTextString[20]  = 'T';
				UsartTextString[21]  = 'O';
				UsartTextString[22] = 'P';
				UsartTextString[23] = '\0';

				counter_X = 0;
				counter_Y = 0;
				break;
			}
#endif
			USART_DecodeTextString(UsartTextString, L6470_TextCommandBundle, (uint8_t*) L6470_DaisyChainSpiTxStruct, (uint8_t*) L6470_DaisyChainSpiRxStruct);
			UsartTextString_1[0] = '\0';
		}

		/* Prepare to receive a text string via USART with UART_IT_RXNE */
		NucleoUsartReceiveIT(&huart2, UsartTextString_1, USARTTEXTSTRINGSIZE);
	}
}

/**
 * @brief  Handle text character just received.
 * @param  huart pointer to a UART_HandleTypeDef structure that contains
 *               the configuration information for the specified UART module.
 * @note To use inside USART2_IRQHandler function.
 */
void USART_ITCharManager(UART_HandleTypeDef* huart) {
	uint8_t UART_Receive_IT_Char;

	UART_Receive_IT_Char = (uint8_t) (huart->Instance->USART_DATA_REGISTER);
	/* Checks the buffer full or retur carriage  */
	if ((huart->RxXferCount == 1) || (UART_Receive_IT_Char == '\r')) {
		huart->RxXferCount += 1;
		huart->pRxBuffPtr -= 1;
		*(huart->pRxBuffPtr) = '\0';

		USART_Transmit(huart, "\n\r");

		while (HAL_IS_BIT_SET(huart->Instance->USART_STATUS_REGISTER, UART_FLAG_RXNE)) {
		}
		__HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);

		/* Check if a transmit process is ongoing or not */
		if (huart->gState == HAL_UART_STATE_BUSY_TX_RX) {
			huart->gState = HAL_UART_STATE_BUSY_TX;
		} else {
			/* Disable the UART Parity Error Interrupt */
			__HAL_UART_DISABLE_IT(huart, UART_IT_PE);

			/* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
			__HAL_UART_DISABLE_IT(huart, UART_IT_ERR);

			huart->gState = HAL_UART_STATE_READY;
		}
	}
}

/**
 * @brief  Print on the PC screen the values of the L6470 Registers.
 * @param  ExpBrd    The addressed Expansion Board
 * @param  L6470_Id  The addressed L6470
 *         
 */
void USART_PrintRegisterValues(uint8_t ExpBrd, uint8_t L6470_Id) {
	uint8_t r;
	uint32_t value;

	/* Initialize the used GPIO for the L6470 nCS related to the addressed X-NUCLEO-IHM02A1 */
	if (BSP_Select(ExpBrd)) {
		USART_Transmit(&huart2, "\n\r");

		USART_Transmit(&huart2, "Values of the registers of L6470_#");
		USART_Transmit(&huart2, num2hex(L6470_Id, HALFBYTE_F));
		USART_Transmit(&huart2, " of ExpBrd_#");
		USART_Transmit(&huart2, num2hex(ExpBrd, HALFBYTE_F));
		USART_Transmit(&huart2, "\n\r\n\r");

		for (r = 0; r < L6470REGIDSIZE; r++) {
			value = L6470_GetParam(L6470_Id, (eL6470_RegId_t) r);

			USART_Transmit(&huart2, L6470_GetRegisterName((eL6470_RegId_t) r));
			USART_Transmit(&huart2, ": ");
			USART_Transmit(&huart2, num2hex(value, DOUBLEWORD_F));
			USART_Transmit(&huart2, "\n\r");
		}

		USART_Transmit(&huart2, "\n\r");
	}
}

/**
 * @brief  Print on the PC screen the values of the L6470 Registers for all
 *         devices mounted on all stacked X-NUCLEO-IHM02A1.
 */
void USART_PrintAllRegisterValues(void) {
	uint8_t ExpBrd; /* to index the expansion board to be addressed */
	uint8_t L6470_Id; /* to index the L6470 into the daisy chain to be addressed */

	for (ExpBrd = EXPBRD_ID(0); ExpBrd <= EXPBRD_ID(EXPBRD_MOUNTED_NR - 1); ExpBrd++) {
		for (L6470_Id = L6470_ID(0); L6470_Id <= L6470_ID(L6470DAISYCHAINSIZE - 1); L6470_Id++) {
			USART_PrintRegisterValues(ExpBrd, L6470_Id);
		}
	}
}

/**
 * @brief  Fill the L6470_DaisyChainMnemonic structure.
 *
 * @note   This structure will contain the assigned names about the L6470
 *         devices inside the daisy chain and theirs related motors.
 * @note   Its size and contents is updated at boot in relation to the L6470DAISYCHAINSIZE.
 */
void Fill_L6470_DaisyChainMnemonic(void) {
	uint8_t i;
	char c1 = '0';
	char c10 = '0';

	for (i = 0; i < L6470DAISYCHAINSIZE; i++) {
		if (i < 10) {
			c10 = '0' + i;
			c1 = '\0';
		} else {
			c10 = '0' + (i / 10);
			c1 = '0' + (i - ((i / 10) * 10));
		}

		L6470_DaisyChainMnemonic[i].L6470IdMnemonic[0] = 'L';
		L6470_DaisyChainMnemonic[i].L6470IdMnemonic[1] = '6';
		L6470_DaisyChainMnemonic[i].L6470IdMnemonic[2] = '4';
		L6470_DaisyChainMnemonic[i].L6470IdMnemonic[3] = '7';
		L6470_DaisyChainMnemonic[i].L6470IdMnemonic[4] = '0';
		L6470_DaisyChainMnemonic[i].L6470IdMnemonic[5] = '#';
		L6470_DaisyChainMnemonic[i].L6470IdMnemonic[6] = c10;
		L6470_DaisyChainMnemonic[i].L6470IdMnemonic[7] = c1;
		L6470_DaisyChainMnemonic[i].L6470IdMnemonic[8] = '\0';
		L6470_DaisyChainMnemonic[i].MotorIdMnemonic[0] = 'M';
		L6470_DaisyChainMnemonic[i].MotorIdMnemonic[1] = c10;
		L6470_DaisyChainMnemonic[i].MotorIdMnemonic[2] = c1;
		L6470_DaisyChainMnemonic[i].MotorIdMnemonic[3] = '\0';
	}
}

/**
 * @brief  Convert a number nbr into a string str with 7 characters.
 * @param  nbr The number to be converted.
 * @param  str The container of the converted number into a text in decimal
 *         format.
 * @note   The decimal digits of the number must be maximum 7 so str has to be
 *         able to store at least 7 characters plus '\0'.
 */
void num2str(uint32_t nbr, uint8_t *str) {
	uint8_t k;
	uint8_t *pstrbuff;
	uint32_t divisor;

	pstrbuff = str;

	/* Reset the text string */
	for (k = 0; k < 7; k++)
		*(pstrbuff + k) = '\0';

	divisor = 1000000;

	if (nbr) // if nbr is different from zero then it is processed
	{
		while (!(nbr / divisor)) {
			divisor /= 10;
		}

		while (divisor >= 10) {
			k = nbr / divisor;
			*pstrbuff++ = '0' + k;
			nbr = nbr - (k * divisor);
			divisor /= 10;
		}
	}

	*pstrbuff++ = '0' + nbr;
	*pstrbuff++ = '\0';
}

/**
 * @brief  Convert an integer number into hexadecimal format.
 *
 * @param  num         The integer number to convert.
 * @param  HexFormat   The output format about hexadecimal number.
 *
 * @retval uint8_t*    The address of the string text for the converted hexadecimal number.
 */
uint8_t* num2hex(uint32_t num, eHexFormat HexFormat) {
	static uint8_t HexValue[8 + 1];
	uint8_t i;
	uint8_t dummy;
	uint8_t HexDigits;

	switch (HexFormat) {
	case HALFBYTE_F:
		HexDigits = 1;
		break;
	case BYTE_F:
		HexDigits = 2;
		break;
	case WORD_F:
		HexDigits = 4;
		break;
	case DOUBLEWORD_F:
		HexDigits = 8;
		break;
	}

	for (i = 0; i < HexDigits; i++) {
		HexValue[i] = '\0';
		dummy = (num & (0x0F << (((HexDigits - 1) - i) * 4))) >> (((HexDigits - 1) - i) * 4);
		if (dummy < 0x0A) {
			HexValue[i] = dummy + '0';
		} else {
			HexValue[i] = (dummy - 0x0A) + 'A';
		}
	}
	HexValue[i] = '\0';

	return HexValue;
}

/**
 * @}
 *//* End of ExampleUsartExportedFunctions */

/**
 * @}
 *//* End of ExampleUsart */

/**
 * @}
 *//* End of MicrosteppingMotor_Example */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
