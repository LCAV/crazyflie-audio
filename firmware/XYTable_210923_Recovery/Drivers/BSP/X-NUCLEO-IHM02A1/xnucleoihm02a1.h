/**
  ******************************************************************************
  * @file       xnucleoihm02a1.h
  * @date       01/10/2014 12:00:00
  * @brief      This file contains definitions, exported variables and function
  *             prototypes related to the X-NUCLEO-IHM02A1.
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
#ifndef __X_NUCLEO_IHM02A1_H
#define __X_NUCLEO_IHM02A1_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "L6470.h"

/**
  * @addtogroup BSP
  * @{
  */

/**
  * @defgroup X-NUCLEO-IHM02A1
  * @brief    Tools to manage the X-NUCLEO-IHM02A1 Expansion Board.
  * @code
  *
  * How to stack up to four X-NUCLEO-IHM02A1 Expansion Boards on a STM32 NUCLEO:
  *
  *                        +-----------------+
  *                        |                 |
  *                        |                 |
  *                        |                 |
  *                        |                 |
  *                        |                 |
  *                     +--|                 |
  *                     |  |  +---+   +---+  |
  *       +-------------+->+--� 1 +-->� 0 +--+>-----+
  *       |  SDI        |  |  +---+   +---+  |  SDO |
  *  +----|-------------+->+----+-------+ (3)|      |
  *  |    |  nCS3       |  +-----------------+      |
  *  |    |          +--|                 |         |
  *  |    |          |  |  +---+   +---+  |         |
  *  |    +----------+->+--� 1 +-->� 0 +--+>--------�
  *  |    |  SDI     |  |  +---+   +---+  |  SDO    |
  *  |+---|----------+->+----+-------+ (2)|         |
  *  ||   |  nCS2    |  +-----------------+         |
  *  ||   |       +--|                 |            |
  *  ||   |       |  |  +---+   +---+  |            |
  *  ||   +-------+->+--� 1 +-->� 0 +--+>-----------�
  *  ||   |  SDI  |  |  +---+   +---+  |  SDO       |
  *  ||+--|-------+->+----+-------+ (1)|            |
  *  |||  |  nCS1 |  +-----------------+            |
  *  |||  |       |                 |               |
  *  |||  |       |  +---+   +---+  |               |
  *  |||  +------>+--� 1 +-->� 0 +--+>--------------�
  *  |||  |  SDI  |  +---+   +---+  |  SDO          |
  *  |||+-|------>+----+-------+ (0)|               |
  *  |||| |  nCS0 +-----------------+               |
  *  |||| |                                         |
  *  |||| |       +-----------------+               |
  *  |||| +------<�                 |<--------------+
  *  ||||    MOSI |                 | MISO
  *  |||+---------�  STM32 NUCLEO   |
  *  |||     nCS0 |                 |
  *  ||+----------�                 |
  *  ||      nCS1 |                 |
  *  |+-----------�                 |
  *  |       nCS2 |                 |
  *  +------------�                 |
  *          nCS3 +-----------------+
  *
  * @endcode
  * @{
  */

/**
  * @defgroup   X-NUCLEO-IHM02A1_FEATURE_SELECTION X-NUCLEO-IHM02A1 Feature Selection
  * @{
  */
   
#define EXPANSION_BOARD_SB23_MOUNTED    //!< Tag about the X-NUCLEO-IHM02A1 with SPI nCS linked to SB23 (OP0)
//#define EXPANSION_BOARD_SB7_MOUNTED     //!< Tag about the X-NUCLEO-IHM02A1 with SPI nCS linked to SB7 (OP1)
//#define EXPANSION_BOARD_SB8_MOUNTED     //!< Tag about the X-NUCLEO-IHM02A1 with SPI nCS linked to SB8 (OP2)
//#define EXPANSION_BOARD_SB9_MOUNTED     //!< Tag about the X-NUCLEO-IHM02A1 with SPI nCS linked to SB9 (OP3)
#if (!defined (EXPANSION_BOARD_SB23_MOUNTED) && !defined (EXPANSION_BOARD_SB7_MOUNTED) &&\
  !defined (EXPANSION_BOARD_SB8_MOUNTED) && !defined (EXPANSION_BOARD_SB9_MOUNTED))
  #error "Please select an option at least!"
#endif

//#define BSP_L6470_CLOCK_SYNCHRO  //!< Uncomment for synchronizing both L6470 clock of all mounted X-NUCLEO-IHM02A1

/**
  * @}
  */ /* End of X-NUCLEO-IHM02A1_FEATURE_SELECTION */

/**
  * @defgroup   X-NUCLEO-IHM02A1_Exported_Macros
  * @brief      X-NUCLEO-IHM02A1 Exported Macros.
  * @{
  */

#define EXPBRD_0                                0   //!< aid for calculating the number of mounted expansion boards, this refers to board with R23 on
#define EXPBRD_1                                0   //!< aid for calculating the number of mounted expansion boards, this refers to board with R25 on
#define EXPBRD_2                                0   //!< aid for calculating the number of mounted expansion boards, this refers to board with R31 on
#define EXPBRD_3                                0   //!< aid for calculating the number of mounted expansion boards, this refers to board with R27 on

#define EXPBRD_MOUNTED_NR_MAX                   1   //!< The max number of mounted X-NUCLEO-IHM02A1

#ifdef EXPANSION_BOARD_SB23_MOUNTED
  #undef EXPBRD_0
  #define EXPBRD_0                              1                     /* Updating */
#endif
#ifdef EXPANSION_BOARD_SB7_MOUNTED
  #undef EXPBRD_1
  #define EXPBRD_1                              1                     /* Updating */
#endif
#ifdef EXPANSION_BOARD_SB8_MOUNTED
  #undef EXPBRD_2
  #define EXPBRD_2                              1                     /* Updating */
#endif
#ifdef EXPANSION_BOARD_SB9_MOUNTED
  #undef EXPBRD_3
  #define EXPBRD_3                              1                     /* Updating */
#endif

#define EXPBRD_MOUNTED_NR                       (EXPBRD_0 + EXPBRD_1 + EXPBRD_2 + EXPBRD_3) //!< The number of mounted X-NUCLEO-IHM02A1

/**
  * @brief  Check the status of the L6470 nBUSY_SYNC pin
  * @retval PinState The input port pin value.
  */
#define BSP_L6470_nBUSYPINSTATUS()   HAL_GPIO_ReadPin(L6470_nBUSY_SYNC_GPIO_PORT, L6470_nBUSY_SYNC_GPIO_PIN)

/**
  * @brief  Check the status of the L6470 nFLAG pin
  * @retval PinState The input port pin value.
  */
#define BSP_L6470_nFLAGPINSTATUS()   HAL_GPIO_ReadPin(L6470_nFLAG_GPIO_PORT, L6470_nFLAG_GPIO_PIN)

/**
  * @}
  */ /* End of X-NUCLEO-IHM02A1_Exported_Macros */

/**
  * @defgroup   X-NUCLEO-IHM02A1_Exported_Functions
  * @brief      X-NUCLEO-IHM02A1 Exported Functions.
  * @{
  */

/**
  * @defgroup   X-NUCLEO-IHM02A1_AppCMDs
  * @brief      The following functions act just on one driver inside the L6470
  *             daisy chain of the selected X-NUCLEO-IHM02A1 Expansion Board.
  *             The command is immediately sent.
  * @{
  */
void BSP_L6470_SetParam(uint8_t ExpBrd_Id, uint8_t L6470_Id, eL6470_RegId_t L6470_RegId, uint32_t Value);
uint32_t BSP_L6470_GetParam(uint8_t ExpBrd_Id, uint8_t L6470_Id, eL6470_RegId_t L6470_RegId);
void BSP_L6470_Run(uint8_t ExpBrd_Id, uint8_t L6470_Id, eL6470_DirId_t L6470_DirId, uint32_t Speed);
void BSP_L6470_StepClock(uint8_t ExpBrd_Id, uint8_t L6470_Id, eL6470_DirId_t L6470_DirId);
void BSP_L6470_Move(uint8_t ExpBrd_Id, uint8_t L6470_Id, eL6470_DirId_t L6470_DirId, uint32_t N_Step);
void BSP_L6470_GoTo(uint8_t ExpBrd_Id, uint8_t L6470_Id, uint32_t AbsPos);
void BSP_L6470_GoToDir(uint8_t ExpBrd_Id, uint8_t L6470_Id, eL6470_DirId_t L6470_DirId, uint32_t AbsPos);
void BSP_L6470_GoUntil(uint8_t ExpBrd_Id, uint8_t L6470_Id, eL6470_ActId_t L6470_ActId, eL6470_DirId_t L6470_DirId, uint32_t Speed);
void BSP_L6470_ReleaseSW(uint8_t ExpBrd_Id, uint8_t L6470_Id, eL6470_ActId_t L6470_ActId, eL6470_DirId_t L6470_DirId);
void BSP_L6470_GoHome(uint8_t ExpBrd_Id, uint8_t L6470_Id);
void BSP_L6470_GoMark(uint8_t ExpBrd_Id, uint8_t L6470_Id);
void BSP_L6470_ResetPos(uint8_t ExpBrd_Id, uint8_t L6470_Id);
void BSP_L6470_ResetDevice(uint8_t ExpBrd_Id, uint8_t L6470_Id);
void BSP_L6470_SoftStop(uint8_t ExpBrd_Id, uint8_t L6470_Id);
void BSP_L6470_HardStop(uint8_t ExpBrd_Id, uint8_t L6470_Id);
void BSP_L6470_SoftHiZ(uint8_t ExpBrd_Id, uint8_t L6470_Id);
void BSP_L6470_HardHiZ(uint8_t ExpBrd_Id, uint8_t L6470_Id);
uint16_t BSP_L6470_GetStatus(uint8_t ExpBrd_Id, uint8_t L6470_Id);
/**
  * @}
  */ /* End of X-NUCLEO-IHM02A1_AppCMDs */

uint8_t* BSP_L6470_PerformPreparedApplicationCommand(uint8_t ExpBrd_Id);
uint8_t BSP_L6470_CheckStatusRegisterFlag(uint8_t ExpBrd_Id, uint8_t L6470_Id, uint8_t L6470_StatusRegisterFlagId);
uint8_t EXPBRD_ID(uint8_t position);
uint8_t BSP_Select(uint8_t ExpBrd_Id);
void BSP_Init(void);
void BSP_L6470_BusySynchEventManager(void);
void BSP_L6470_FlagEventManager(void);
void BSP_EmergencyStop(void);

void BSP_0_Config(MotorParameterData_t *MotorParameterData);
void BSP_1_Config(MotorParameterData_t *MotorParameterData);
void BSP_2_Config(MotorParameterData_t *MotorParameterData);
void BSP_3_Config(MotorParameterData_t *MotorParameterData);
void BSP_L6470_0_0_Config(MotorParameterData_t *MotorParameterData);
void BSP_L6470_0_1_Config(MotorParameterData_t *MotorParameterData);
void BSP_L6470_1_0_Config(MotorParameterData_t *MotorParameterData);
void BSP_L6470_1_1_Config(MotorParameterData_t *MotorParameterData);
void BSP_L6470_2_0_Config(MotorParameterData_t *MotorParameterData);
void BSP_L6470_2_1_Config(MotorParameterData_t *MotorParameterData);
void BSP_L6470_3_0_Config(MotorParameterData_t *MotorParameterData);
void BSP_L6470_3_1_Config(MotorParameterData_t *MotorParameterData);
StepperMotorBoardHandle_t *BSP_GetExpansionBoardHandle(uint8_t id);
void BSP_Config(StepperMotorBoardHandle_t *StepperMotorBoardHandle, MotorParameterData_t *MotorParameterData);

/**
  * @}
  */ /* End of X-NUCLEO-IHM02A1_Exported_Functions */

/**
  * @}
  */ /* End of X-NUCLEO-IHM02A1 */

/**
  * @}
  */ /* End of BSP */

#ifdef __cplusplus
}
#endif

#endif /* __X_NUCLEO_IHM02A1_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
