/**
  ******************************************************************************
  * @file       xnucleoihm02a1.c
  * @date       01/10/2014 12:00:00
  * @brief      This file provides set of firmware functions to manage the
  *             X-NUCLEO-IHM02A1.
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

#include "xnucleoihm02a1.h"
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
  * @defgroup   X-NUCLEO-IHM02A1_Private_Variables
  * @brief      X-NUCLEO-IHM02A1 Private Variables.
  * @{
  */

StepperMotorBoardCommand_t XNUCLEOIHM02A1Command =
{
  BSP_L6470_SetParam,
  BSP_L6470_GetParam,
  BSP_L6470_Run,
  BSP_L6470_StepClock,
  BSP_L6470_Move,
  BSP_L6470_GoTo,
  BSP_L6470_GoToDir,
  BSP_L6470_GoUntil,
  BSP_L6470_ReleaseSW,
  BSP_L6470_GoHome,
  BSP_L6470_GoMark,
  BSP_L6470_ResetPos,
  BSP_L6470_ResetDevice,
  BSP_L6470_SoftStop,
  BSP_L6470_HardStop,
  BSP_L6470_SoftHiZ,
  BSP_L6470_HardHiZ,
  BSP_L6470_GetStatus,  
  BSP_L6470_CheckStatusRegisterFlag,
  L6470_PerformPreparedApplicationCommand,
};
  
/**
  * @brief The data structure for all further instances to stepper motor drivers.
  */
StepperMotorDriverHandle_t L6470DrvMtx[4][2] =
{
  {
    {
      0,
      BSP_L6470_0_0_Config,
      &L6470Command,
    },
    {
      1,
      BSP_L6470_0_1_Config,
      &L6470Command,
    },
  },
  {
    {
      0,
      BSP_L6470_1_0_Config,
      &L6470Command,
    },
    {
      1,
      BSP_L6470_1_1_Config,
      &L6470Command,
    },
  },
  {
    {
      0,
      BSP_L6470_2_0_Config,
      &L6470Command,
    },
    {
      1,
      BSP_L6470_2_1_Config,
      &L6470Command,
    },
  },
  {
    {
      0,
      BSP_L6470_3_0_Config,
      &L6470Command,
    },
    {
      1,
      BSP_L6470_3_1_Config,
      &L6470Command,
    },
  },
};

/**
  * @brief The data structure for all further instances to expansio board drivers.
  */
StepperMotorBoardHandle_t XNUCLEOIHM02A1Drv[4] =
{
  {
    0,
    BSP_0_Config,
    &XNUCLEOIHM02A1Command,
	{
	  &L6470DrvMtx[0][0],
	  &L6470DrvMtx[0][1],
	},
    BSP_Select,
  },
  {
    1,
    BSP_1_Config,
    &XNUCLEOIHM02A1Command,
	{
      &L6470DrvMtx[1][0],
	  &L6470DrvMtx[1][1],
	},
    BSP_Select,
  },
  {
    2,
    BSP_2_Config,
    &XNUCLEOIHM02A1Command,
	{
	  &L6470DrvMtx[2][0],
	  &L6470DrvMtx[2][1],
	},
    BSP_Select,
  },
  {
    3,
    BSP_3_Config,
    &XNUCLEOIHM02A1Command,
	{
	  &L6470DrvMtx[3][0],
	  &L6470DrvMtx[3][1],
	},
    BSP_Select,
  },
};

/**
  * @brief  The data structure to store the GPIO information about the L6470 nSTBY_nRST.
  */
sL6470_GPIO L6470_nSTBY_nRST_GPIO = {L6470_nSTBY_nRST_GPIO_CLK_ENABLE, L6470_nSTBY_nRST_GPIO_PORT, L6470_nSTBY_nRST_GPIO_PIN};
/**
  * @brief  The data structure to store the GPIO information about the L6470 nBUSY_SYNC.
  */
sL6470_GPIO L6470_nBUSY_SYNC_GPIO = {L6470_nBUSY_SYNC_GPIO_CLK_ENABLE, L6470_nBUSY_SYNC_GPIO_PORT, L6470_nBUSY_SYNC_GPIO_PIN};
/**
  * @brief  The data structure to store the GPIO information about the L6470 nFLAG.
  */
sL6470_GPIO L6470_nFLAG_GPIO = {L6470_nFLAG_GPIO_CLK_ENABLE, L6470_nFLAG_GPIO_PORT, L6470_nFLAG_GPIO_PIN};
/**
  * @brief  The data structure to store the actual GPIO information about the L6470 nCS.
  */
sL6470_GPIO L6470_nCS_GPIO;
/**
  * @brief  The data structure to store the GPIO information about the L6470 nCS.
  * @note   The expansion board will numbered from 0 up to (EXPBRD_MOUNTED_NR-1)
  *         following the ordering of nCS driven by PA4, PA10, PB6 and PB4.
  */
sL6470_GPIO L6470_nCS_GPIO_Array[EXPBRD_MOUNTED_NR] = {
#ifdef EXPANSION_BOARD_SB23_MOUNTED
  {
    L6470_nCS_OP0_GPIO_CLK_ENABLE,
    L6470_nCS_OP0_GPIO_PORT,
    L6470_nCS_OP0_GPIO_PIN,
  },
#endif
#ifdef EXPANSION_BOARD_SB7_MOUNTED
  {
    L6470_nCS_OP1_GPIO_CLK_ENABLE,
    L6470_nCS_OP1_GPIO_PORT,
    L6470_nCS_OP1_GPIO_PIN,
  },
#endif  
#ifdef EXPANSION_BOARD_SB8_MOUNTED
  {
    L6470_nCS_OP2_GPIO_CLK_ENABLE,
    L6470_nCS_OP2_GPIO_PORT,
    L6470_nCS_OP2_GPIO_PIN,
  },
#endif  
#ifdef EXPANSION_BOARD_SB9_MOUNTED
  {
    L6470_nCS_OP3_GPIO_CLK_ENABLE,
    L6470_nCS_OP3_GPIO_PORT,
    L6470_nCS_OP3_GPIO_PIN,
  },
#endif
};

/**
  * @brief  The data structure to store the Status Register for each L6470 mounted on each X-NUCLEO-IHM02A1.
  */
sL6470_StatusRegister_t StatusBuffer[EXPBRD_MOUNTED_NR][L6470DAISYCHAINSIZE];

/**
  * @}
  */ /* End of X-NUCLEO-IHM02A1_Private_Variables */

/**
  * @defgroup   X-NUCLEO-IHM02A1_Private_Functions
  * @brief      X-NUCLEO-IHM02A1 Private Functions.
  * @{
  */

void BSP_NUCLEO_GPIO_Init(void);
void BSP_L6470_Init(void);
void BSP_IRQ(FunctionalState fs);

/**
  * @}
  */ /* End of X-NUCLEO-IHM02A1_Private_Functions */

/**
  * @addtogroup X-NUCLEO-IHM02A1_Private_Functions
  * @{
  */

/**
  * @brief  Initialize the NUCLEO GPIO used by the X-NUCLEO-IHM02A1.
  * 
  * @note   The following pins will be initialized:
  *         - nSTBY_nRST (Output Open Drain)
  *         - nBUSY_SYNC (Input with IRQ on falling edge)
  *         - nFLAG (Input with IRQ on falling edge)
  *         - one or more nCS (Output Push-Pull)
  */
void BSP_NUCLEO_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  uint8_t ExpBrd;
  
  /* Initialize X-NUCLEO-IHM02A1 nSTBY_nRST pin */
  GPIO_CLK_ENABLE(L6470_nSTBY_nRST_GPIO.gpio_clk_enable);
  GPIO_InitStruct.Pin = L6470_nSTBY_nRST_GPIO.pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(L6470_nSTBY_nRST_GPIO.port, &GPIO_InitStruct);
  
  /* Initialize the X-NUCLEO-IHM02A1 nBUSY_SYNC pin */
  GPIO_CLK_ENABLE(L6470_nBUSY_SYNC_GPIO.gpio_clk_enable);
  GPIO_InitStruct.Pin = L6470_nBUSY_SYNC_GPIO.pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(L6470_nBUSY_SYNC_GPIO.port, &GPIO_InitStruct);

  /* Sets the priority grouping field */
  HAL_NVIC_SetPriority(L6470_nBUSY_SYNC_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(L6470_nBUSY_SYNC_IRQn);

  /* Initialize X-NUCLEO-IHM02A1 nFLAG pin */
  GPIO_CLK_ENABLE(L6470_nFLAG_GPIO.gpio_clk_enable);
  GPIO_InitStruct.Pin = L6470_nFLAG_GPIO.pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(L6470_nFLAG_GPIO.port, &GPIO_InitStruct);

  /* Sets the priority grouping field */
  HAL_NVIC_SetPriority(L6470_nFLAG_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(L6470_nFLAG_IRQn);
  
  /* Initialize X-NUCLEO-IHM02A1 nCS pin */
  for (ExpBrd = EXPBRD_ID(0); ExpBrd <= EXPBRD_ID(EXPBRD_MOUNTED_NR-1); ExpBrd++)
  {
    GPIO_CLK_ENABLE(L6470_nCS_GPIO_Array[ExpBrd].gpio_clk_enable);
    GPIO_InitStruct.Pin = L6470_nCS_GPIO_Array[ExpBrd].pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(L6470_nCS_GPIO_Array[ExpBrd].port, &GPIO_InitStruct);
  }
}

/**
  * @brief  Initialize the X-NUCLEO-IHM02A1 mounted board and theirs L6470 devices.
  */
void BSP_L6470_Init(void)
{
  uint8_t ExpBrd;                     /* to index the expansion board to be addressed */
  uint8_t L6470_Id;                   /* to index the L6470 to be addressed */
  
  /* Disable the L6470 */
  L6470_DISABLE();
  
  /* Enable the L6470 */
  L6470_ENABLE();
  
  /* Initialize all mounted L6470 for each stacked X-NUCLEO-IHM02A1 */
  for (ExpBrd = EXPBRD_ID(0); ExpBrd <= EXPBRD_ID(EXPBRD_MOUNTED_NR-1); ExpBrd++)
  {
    /* Select the used GPIO for the L6470 nCS mounted on the addressed X-NUCLEO-IHM02A1 */
    BSP_Select(ExpBrd);
    
    /* Disable SPI communication for L6470 */
    L6470_nCS_HIGH();
    
    for (L6470_Id = L6470_ID(0); L6470_Id<=L6470_ID(L6470DAISYCHAINSIZE-1); L6470_Id++)
    {
      /* Reset the devices */
      L6470_ResetDevice(L6470_Id);
      
      /* Reset all Status Register Flags */
      L6470_GetStatus(L6470_Id);
    }
  }
 
  /* Select the default X-NUCLEO-IHM02A1 to be addressed by SPI */
  BSP_Select(EXPBRD_ID(0));
}

/**
  * @brief  Enable the IRQ for the nBUSY_SYNC and the nFLAG of the X-NUCLEO-IHM02A1
  */
void BSP_IRQ(FunctionalState fs)
{
  if(fs == ENABLE)
  {
    HAL_NVIC_EnableIRQ(L6470_nBUSY_SYNC_IRQn);
    HAL_NVIC_EnableIRQ(L6470_nFLAG_IRQn);
  }
  else
  {
    HAL_NVIC_DisableIRQ(L6470_nBUSY_SYNC_IRQn);
    HAL_NVIC_DisableIRQ(L6470_nFLAG_IRQn);
  }
}

/**
  * @}
  */ /* End of X-NUCLEO-IHM02A1_Private_Functions */

/**
  * @addtogroup X-NUCLEO-IHM02A1_Exported_Functions
  * @{
  */

/**
  * @addtogroup X-NUCLEO-IHM02A1_AppCMDs
  * @{
  */

/**
  * @brief  Perform the L6470_SetParam Application Command for the selected device on the selected Expansion Board.
  * @param  ExpBrd_Id       The addressed Expansion Board
  * @param  L6470_Id        The L6470 identifier inside the daisy chain
  * @param  L6470_RegId     The L6470 Register identifier
  * @param  Value           The new value for the L6470 Register
  */
void BSP_L6470_SetParam(uint8_t ExpBrd_Id, uint8_t L6470_Id, eL6470_RegId_t L6470_RegId, uint32_t Value)
{
  /* Initialize the used GPIO for the L6470 nCS related to the addressed X-NUCLEO-IHM02A1 */
  if (BSP_Select(ExpBrd_Id))
  {
    /* Perform the related L6470 Application Command */
    L6470_SetParam(L6470_Id, L6470_RegId, Value);
  }
}

/**
  * @brief  Perform the L6470_GetParam Application Command for the selected device on the selected Expansion Board.
  * @param  ExpBrd_Id       The addressed Expansion Board
  * @param  L6470_Id        The L6470 identifier inside the daisy chain
  * @param  L6470_RegId     The L6470 Register identifier
  * @retval uint32_t        The value of the L6470 Register
  */
uint32_t BSP_L6470_GetParam(uint8_t ExpBrd_Id, uint8_t L6470_Id, eL6470_RegId_t L6470_RegId)
{
  /* Initialize the used GPIO for the L6470 nCS related to the addressed X-NUCLEO-IHM02A1 */
  if (BSP_Select(ExpBrd_Id))
  {
    /* Perform the related L6470 Application Command */
    return L6470_GetParam(L6470_Id, L6470_RegId);
  }
  else
    return 0xFF;
}

/**
  * @brief  Perform the L6470_Run Application Command for the selected device on the selected Expansion Board.
  * @param  ExpBrd_Id       The addressed Expansion Board
  * @param  L6470_Id        The L6470 identifier inside the daisy chain
  * @param  L6470_DirId     The L6470 Direction identifier
  * @param  Speed           The new value about the speed
  */
void BSP_L6470_Run(uint8_t ExpBrd_Id, uint8_t L6470_Id, eL6470_DirId_t L6470_DirId, uint32_t Speed)
{
  /* Initialize the used GPIO for the L6470 nCS related to the addressed X-NUCLEO-IHM02A1 */
  if (BSP_Select(ExpBrd_Id))
  {
    /* Perform the related L6470 Application Command */
    L6470_Run(L6470_Id, L6470_DirId, Speed);
  }
}

/**
  * @brief  Perform the L6470_StepClock Application Command for the selected device on the selected Expansion Board.
  * @param  ExpBrd_Id       The addressed Expansion Board
  * @param  L6470_Id        The L6470 identifier inside the daisy chain
  * @param  L6470_DirId     The L6470 Direction identifier
  */
void BSP_L6470_StepClock(uint8_t ExpBrd_Id, uint8_t L6470_Id, eL6470_DirId_t L6470_DirId)
{
  /* Initialize the used GPIO for the L6470 nCS related to the addressed X-NUCLEO-IHM02A1 */
  if (BSP_Select(ExpBrd_Id))
  {
    /* Perform the related L6470 Application Command */
    L6470_StepClock(L6470_Id, L6470_DirId);
  }
}

/**
  * @brief  Perform the L6470_Move Application Command for the selected device on the selected Expansion Board.
  * @param  ExpBrd_Id       The addressed Expansion Board
  * @param  L6470_Id        The L6470 identifier inside the daisy chain
  * @param  L6470_DirId     The L6470 Direction identifier
  * @param  N_Step          The number of step about the movement to be performed
  */
void BSP_L6470_Move(uint8_t ExpBrd_Id, uint8_t L6470_Id, eL6470_DirId_t L6470_DirId, uint32_t N_Step)
{
  /* Initialize the used GPIO for the L6470 nCS related to the addressed X-NUCLEO-IHM02A1 */
  if (BSP_Select(ExpBrd_Id))
  {
    /* Perform the related L6470 Application Command */
    L6470_Move(L6470_Id, L6470_DirId, N_Step);
  }
}

/**
  * @brief  Perform the L6470_GoTo Application Command for the selected device on the selected Expansion Board.
  * @param  ExpBrd_Id       The addressed Expansion Board
  * @param  L6470_Id        The L6470 identifier inside the daisy chain
  * @param  AbsPos          The L6470 absolute position to be reached
  */
void BSP_L6470_GoTo(uint8_t ExpBrd_Id, uint8_t L6470_Id, uint32_t AbsPos)
{
  /* Initialize the used GPIO for the L6470 nCS related to the addressed X-NUCLEO-IHM02A1 */
  if (BSP_Select(ExpBrd_Id))
  {
    /* Perform the related L6470 Application Command */
    L6470_GoTo(L6470_Id, AbsPos);
  }
}

/**
  * @brief  Perform the L6470_GoToDir Application Command for the selected device on the selected Expansion Board.
  * @param  ExpBrd_Id       The addressed Expansion Board
  * @param  L6470_Id        The L6470 identifier inside the daisy chain
  * @param  L6470_DirId     The L6470 Direction identifier
  * @param  AbsPos          The L6470 absolute position to be reached
  */
void BSP_L6470_GoToDir(uint8_t ExpBrd_Id, uint8_t L6470_Id, eL6470_DirId_t L6470_DirId, uint32_t AbsPos)
{
  /* Initialize the used GPIO for the L6470 nCS related to the addressed X-NUCLEO-IHM02A1 */
  if (BSP_Select(ExpBrd_Id))
  {
    /* Perform the related L6470 Application Command */
    L6470_GoToDir(L6470_Id, L6470_DirId, AbsPos);
  }
}

/**
  * @brief  Perform the L6470_GoUntil Application Command for the selected device on the selected Expansion Board.
  * @param  ExpBrd_Id       The addressed Expansion Board
  * @param  L6470_Id        The L6470 identifier inside the daisy chain
  * @param  L6470_ActId     The L6470 Action identifier about ABS_POS register
  * @param  L6470_DirId     The L6470 Direction identifier
  * @param  Speed           The new value about the speed
  */
void BSP_L6470_GoUntil(uint8_t ExpBrd_Id, uint8_t L6470_Id, eL6470_ActId_t L6470_ActId, eL6470_DirId_t L6470_DirId, uint32_t Speed)
{
  /* Initialize the used GPIO for the L6470 nCS related to the addressed X-NUCLEO-IHM02A1 */
  if (BSP_Select(ExpBrd_Id))
  {
    /* Perform the related L6470 Application Command */
    L6470_GoUntil(L6470_Id, L6470_ActId, L6470_DirId, Speed);
  }
}

/**
  * @brief  Perform the L6470_ReleaseSW Application Command for the selected device on the selected Expansion Board.
  * @param  ExpBrd_Id       The addressed Expansion Board
  * @param  L6470_Id        The L6470 identifier inside the daisy chain
  * @param  L6470_ActId     The L6470 Action identifier about ABS_POS register
  * @param  L6470_DirId     The L6470 Direction identifier
  */
void BSP_L6470_ReleaseSW(uint8_t ExpBrd_Id, uint8_t L6470_Id, eL6470_ActId_t L6470_ActId, eL6470_DirId_t L6470_DirId)
{
  /* Initialize the used GPIO for the L6470 nCS related to the addressed X-NUCLEO-IHM02A1 */
  if (BSP_Select(ExpBrd_Id))
  {
    /* Perform the related L6470 Application Command */
    L6470_ReleaseSW(L6470_Id, L6470_ActId, L6470_DirId);
  }
}

/**
  * @brief  Perform the L6470_GoHome Application Command for the selected device on the selected Expansion Board.
  * @param  ExpBrd_Id       The addressed Expansion Board
  * @param  L6470_Id        The L6470 identifier inside the daisy chain
  */
void BSP_L6470_GoHome(uint8_t ExpBrd_Id, uint8_t L6470_Id)
{
  /* Initialize the used GPIO for the L6470 nCS related to the addressed X-NUCLEO-IHM02A1 */
  if (BSP_Select(ExpBrd_Id))
  {
    /* Perform the related L6470 Application Command */
    L6470_GoHome(L6470_Id);
  }
}

/**
  * @brief  Perform the L6470_GoMark Application Command for the selected device on the selected Expansion Board.
  * @param  ExpBrd_Id       The addressed Expansion Board
  * @param  L6470_Id        The L6470 identifier inside the daisy chain
  */
void BSP_L6470_GoMark(uint8_t ExpBrd_Id, uint8_t L6470_Id)
{
  /* Initialize the used GPIO for the L6470 nCS related to the addressed X-NUCLEO-IHM02A1 */
  if (BSP_Select(ExpBrd_Id))
  {
    /* Perform the related L6470 Application Command */
    L6470_GoMark(L6470_Id);
  }
}

/**
  * @brief  Perform the L6470_ResetPos Application Command for the selected device on the selected Expansion Board.
  * @param  ExpBrd_Id       The addressed Expansion Board
  * @param  L6470_Id        The L6470 identifier inside the daisy chain
  */
void BSP_L6470_ResetPos(uint8_t ExpBrd_Id, uint8_t L6470_Id)
{
  /* Initialize the used GPIO for the L6470 nCS related to the addressed X-NUCLEO-IHM02A1 */
  if (BSP_Select(ExpBrd_Id))
  {
    /* Perform the related L6470 Application Command */
    L6470_ResetPos(L6470_Id);
  }
}

/**
  * @brief  Perform the L6470_ResetDevice Application Command for the selected device on the selected Expansion Board.
  * @param  ExpBrd_Id       The addressed Expansion Board
  * @param  L6470_Id        The L6470 identifier inside the daisy chain
  */
void BSP_L6470_ResetDevice(uint8_t ExpBrd_Id, uint8_t L6470_Id)
{
  /* Initialize the used GPIO for the L6470 nCS related to the addressed X-NUCLEO-IHM02A1 */
  if (BSP_Select(ExpBrd_Id))
  {
    /* Perform the related L6470 Application Command */
    L6470_ResetDevice(L6470_Id);
  }
}

/**
  * @brief  Perform the L6470_SoftStop Application Command for the selected device on the selected Expansion Board.
  * @param  ExpBrd_Id       The addressed Expansion Board
  * @param  L6470_Id        The L6470 identifier inside the daisy chain
  */
void BSP_L6470_SoftStop(uint8_t ExpBrd_Id, uint8_t L6470_Id)
{
  /* Initialize the used GPIO for the L6470 nCS related to the addressed X-NUCLEO-IHM02A1 */
  if (BSP_Select(ExpBrd_Id))
  {
    /* Perform the related L6470 Application Command */
    L6470_SoftStop(L6470_Id);
  }
}

/**
  * @brief  Perform the L6470_HardStop Application Command for the selected device on the selected Expansion Board.
  * @param  ExpBrd_Id       The addressed Expansion Board
  * @param  L6470_Id        The L6470 identifier inside the daisy chain
  */
void BSP_L6470_HardStop(uint8_t ExpBrd_Id, uint8_t L6470_Id)
{
  /* Initialize the used GPIO for the L6470 nCS related to the addressed X-NUCLEO-IHM02A1 */
  if (BSP_Select(ExpBrd_Id))
  {
    /* Perform the related L6470 Application Command */
    L6470_HardStop(L6470_Id);
  }
}

/**
  * @brief  Perform the L6470_SoftHiZ Application Command for the selected device on the selected Expansion Board.
  * @param  ExpBrd_Id       The addressed Expansion Board
  * @param  L6470_Id        The L6470 identifier inside the daisy chain
  */
void BSP_L6470_SoftHiZ(uint8_t ExpBrd_Id, uint8_t L6470_Id)
{
  /* Initialize the used GPIO for the L6470 nCS related to the addressed X-NUCLEO-IHM02A1 */
  if (BSP_Select(ExpBrd_Id))
  {
    /* Perform the related L6470 Application Command */
    L6470_SoftHiZ(L6470_Id);
  }
}

/**
  * @brief  Perform the L6470_HardHiZ Application Command for the selected device on the selected Expansion Board.
  * @param  ExpBrd_Id       The addressed Expansion Board
  * @param  L6470_Id        The L6470 identifier inside the daisy chain
  */
void BSP_L6470_HardHiZ(uint8_t ExpBrd_Id, uint8_t L6470_Id)
{
  /* Initialize the used GPIO for the L6470 nCS related to the addressed X-NUCLEO-IHM02A1 */
  if (BSP_Select(ExpBrd_Id))
  {
    /* Perform the related L6470 Application Command */
    L6470_HardHiZ(L6470_Id);
  }
}

/**
  * @brief  Perform the L6470_GetStatus Application Command for the selected device on the selected Expansion Board.
  * @param  ExpBrd_Id       The addressed Expansion Board
  * @param  L6470_Id        The L6470 identifier inside the daisy chain
  * @retval uint16_t        The value of the L6470 Status Register
  */
uint16_t BSP_L6470_GetStatus(uint8_t ExpBrd_Id, uint8_t L6470_Id)
{
  /* Initialize the used GPIO for the L6470 nCS related to the addressed X-NUCLEO-IHM02A1 */
  if (BSP_Select(ExpBrd_Id))
  {
    /* Perform the related L6470 Application Command */
    return L6470_GetStatus(L6470_Id);
  }
  else
    return 0;
}

/**
  * @}
  */ /* End of X-NUCLEO-IHM02A1_AppCMDs */

/**
  * @brief  Perform the already prepared L6470 application commands (@ref L6470_AppCMDs_ToBePrepared) for the devices
  *         placed on the selected Expansion Board.
  * @param  ExpBrd_Id       The addressed Expansion Board.
  * @retval uint8_t*        The pointer to the matrix containing the received data.
  */
uint8_t* BSP_L6470_PerformPreparedApplicationCommand(uint8_t ExpBrd_Id)
{
  /* Initialize the used GPIO for the L6470 nCS related to the addressed X-NUCLEO-IHM02A1 */
  if (BSP_Select(ExpBrd_Id))
  {
    /* Perform the related L6470 Application Command */
    return L6470_PerformPreparedApplicationCommand();
  }
  else
    return NULL;
}

/**
  * @brief  Check the selected flag of the Status Register for the selected device on the selected Expansion Board.
  * @param  ExpBrd_Id                   The addressed Expansion Board.
  * @param  L6470_Id                    The L6470 identifier inside the daisy chain
  * @param  L6470_StatusRegisterFlagId  The L6470 Status Register Flag identifier
  * @retval uint8_t                     The actual flag status.
  */
uint8_t BSP_L6470_CheckStatusRegisterFlag(uint8_t ExpBrd_Id, uint8_t L6470_Id, uint8_t L6470_StatusRegisterFlagId)
{
  /* Initialize the used GPIO for the L6470 nCS related to the addressed X-NUCLEO-IHM02A1 */
  if (BSP_Select(ExpBrd_Id))
  {
    /* Check the flag of the L6470 Register Status related to the device mounted on the addressed X-NUCLEO-IHM02A1 */
    return L6470_CheckStatusRegisterFlag(L6470_Id, L6470_StatusRegisterFlagId);
  }
  else
    return 0xFF;
}

/**
  * @brief  Address the X-NUCLEO-IHM02A1 inside the stacked expansion boards.
  * @param  position The X-NUCLEO-IHM02A1 related the selected nCS pin
  * @retval id The X-NUCLEO-IHM02A1 identifier
  *
  * @note The first accepted position is '0'.
  * @note If the position is not allowable the returned value is 0xFF.
  */
uint8_t EXPBRD_ID(uint8_t position)
{
  if (position < EXPBRD_MOUNTED_NR)
  {
    return (position);
  }
  else
  {
    return 0xFF;
  }
}

/**
  * @brief  Select the used SPI nCS mounted on the addressed X-NUCLEO-IHM02A1.
  * @param  ExpBrd_Id   The identifier of the Expansion Board to be addressed.
  * @retval FlagStatus  SET or RESET related to the result.
  */
uint8_t BSP_Select(uint8_t ExpBrd_Id)
{
  if (ExpBrd_Id < EXPBRD_MOUNTED_NR)
  {
    L6470_nCS_GPIO.gpio_clk_enable = L6470_nCS_GPIO_Array[ExpBrd_Id].gpio_clk_enable;
    L6470_nCS_GPIO.port = L6470_nCS_GPIO_Array[ExpBrd_Id].port;
    L6470_nCS_GPIO.pin = L6470_nCS_GPIO_Array[ExpBrd_Id].pin;
    return SET;
  }
  else
    return RESET;
}

/**
  * @brief  X-NUCLEO-IHM02A1 initialization
  */
void BSP_Init(void)
{
  /* Select the default L6470_nCS_GPIO */
  BSP_Select(EXPBRD_ID(0));
  
  /* Initialize the NUCLEO GPIO used by the X-NUCLEO-IHM02A1 */
  BSP_NUCLEO_GPIO_Init();
  
  /* Initialize the L6470 devices mounted on the X-NUCLEO-IHM02A1 */
  BSP_L6470_Init();

  /* Enable the IRQ for the nBUSY_SYNC and the nFLAG of the X-NUCLEO-IHM02A1*/
  BSP_IRQ(ENABLE);
}

/**
  * @brief  This function manages the busy-synch event.
  */
void BSP_L6470_BusySynchEventManager(void)
{
  __NOP();
}

  /**
  * @brief  This function manages the status flag event.
  */
void BSP_L6470_FlagEventManager(void)
{
  __NOP();
}

/**
  * @brief  This function will stop all motors connected to all stacked expansion board.
  */
void BSP_EmergencyStop(void)
{
  /* Disable the L6470 */
  L6470_DISABLE();
  
  while(1);
}

/**
 * @brief   Initialize the data structure related to the stepper motor board #0.
 * @param   MotorParameterData_t* The pointer to the related parameter data structure.
 */
void BSP_0_Config(MotorParameterData_t *MotorParameterData)
{
  StepperMotorBoardHandle_t *StepperMotorBoardHandle;
  
  StepperMotorBoardHandle = &XNUCLEOIHM02A1Drv[0];
  BSP_Config(StepperMotorBoardHandle, MotorParameterData);
}

/**
 * @brief   Initialize the data structure related to the stepper motor board #1.
 * @param   MotorParameterData_t* The pointer to the related parameter data structure.
 */
void BSP_1_Config(MotorParameterData_t *MotorParameterData)
{
  StepperMotorBoardHandle_t *StepperMotorBoardHandle;
  
  StepperMotorBoardHandle = &XNUCLEOIHM02A1Drv[1];
  BSP_Config(StepperMotorBoardHandle, MotorParameterData);
}

/**
 * @brief   Initialize the data structure related to the stepper motor board #2.
 * @param   MotorParameterData_t* The pointer to the related parameter data structure.
 */
void BSP_2_Config(MotorParameterData_t *MotorParameterData)
{
  StepperMotorBoardHandle_t *StepperMotorBoardHandle;
  
  StepperMotorBoardHandle = &XNUCLEOIHM02A1Drv[2];
  BSP_Config(StepperMotorBoardHandle, MotorParameterData);
}

/**
 * @brief   Initialize the data structure related to the stepper motor board #3.
 * @param   MotorParameterData_t* The pointer to the related parameter data structure.
 */
void BSP_3_Config(MotorParameterData_t *MotorParameterData)
{
  StepperMotorBoardHandle_t *StepperMotorBoardHandle;
  
  StepperMotorBoardHandle = &XNUCLEOIHM02A1Drv[3];
  BSP_Config(StepperMotorBoardHandle, MotorParameterData);
}

/**
 * @brief   Initialize the data structure related to the stepper motor driver #0
 *          of the expansion board #0.
 * @param   MotorParameterData_t* The pointer to the related parameter data structure.
*/
void BSP_L6470_0_0_Config(MotorParameterData_t *MotorParameterData)
{
  StepperMotorDriverHandle_t *StepperMotorDriverHandle;
  
  StepperMotorDriverHandle = &L6470DrvMtx[0][0];
  L6470_Config(StepperMotorDriverHandle, MotorParameterData);
}

/**
 * @brief   Initialize the data structure related to the stepper motor driver #1
 *          of the expansion board #0.
 * @param   MotorParameterData_t* The pointer to the related parameter data structure.
 */
void BSP_L6470_0_1_Config(MotorParameterData_t *MotorParameterData)
{
  StepperMotorDriverHandle_t *StepperMotorDriverHandle;
  
  StepperMotorDriverHandle = &L6470DrvMtx[0][1];
  L6470_Config(StepperMotorDriverHandle, MotorParameterData);
}

/**
 * @brief   Initialize the data structure related to the stepper motor driver #0
 *          of the expansion board #1.
 * @param   MotorParameterData_t* The pointer to the related parameter data structure.
 */
void BSP_L6470_1_0_Config(MotorParameterData_t *MotorParameterData)
{
  StepperMotorDriverHandle_t *StepperMotorDriverHandle;
  
  StepperMotorDriverHandle = &L6470DrvMtx[1][0];
  L6470_Config(StepperMotorDriverHandle, MotorParameterData);
}

/**
 * @brief   Initialize the data structure related to the stepper motor driver #1
 *          of the expansion board #1.
 * @param   MotorParameterData_t* The pointer to the related parameter data structure.
 */
void BSP_L6470_1_1_Config(MotorParameterData_t *MotorParameterData)
{
  StepperMotorDriverHandle_t *StepperMotorDriverHandle;
  
  StepperMotorDriverHandle = &L6470DrvMtx[1][1];
  L6470_Config(StepperMotorDriverHandle, MotorParameterData);
}

/**
 * @brief   Initialize the data structure related to the stepper motor driver #0
 *          of the expansion board #2.
 * @param   MotorParameterData_t* The pointer to the related parameter data structure.
 */
void BSP_L6470_2_0_Config(MotorParameterData_t *MotorParameterData)
{
  StepperMotorDriverHandle_t *StepperMotorDriverHandle;
  
  StepperMotorDriverHandle = &L6470DrvMtx[2][0];
  L6470_Config(StepperMotorDriverHandle, MotorParameterData);
}

/**
 * @brief   Initialize the data structure related to the stepper motor driver #1
 *          of the expansion board #2.
 * @param   MotorParameterData_t* The pointer to the related parameter data structure.
 */
void BSP_L6470_2_1_Config(MotorParameterData_t *MotorParameterData)
{
  StepperMotorDriverHandle_t *StepperMotorDriverHandle;
  
  StepperMotorDriverHandle = &L6470DrvMtx[2][1];
  L6470_Config(StepperMotorDriverHandle, MotorParameterData);
}

/**
 * @brief   Initialize the data structure related to the stepper motor driver #0
 *          of the expansion board #3.
 * @param   MotorParameterData_t* The pointer to the related parameter data structure.
 */
void BSP_L6470_3_0_Config(MotorParameterData_t *MotorParameterData)
{
  StepperMotorDriverHandle_t *StepperMotorDriverHandle;
  
  StepperMotorDriverHandle = &L6470DrvMtx[3][0];
  L6470_Config(StepperMotorDriverHandle, MotorParameterData);
}

/**
 * @brief   Initialize the data structure related to the stepper motor driver #1
 *          of the expansion board #3.
 * @param   MotorParameterData_t* The pointer to the related parameter data structure.
 */
void BSP_L6470_3_1_Config(MotorParameterData_t *MotorParameterData)
{
  StepperMotorDriverHandle_t *StepperMotorDriverHandle;
  
  StepperMotorDriverHandle = &L6470DrvMtx[3][1];
  L6470_Config(StepperMotorDriverHandle, MotorParameterData);
}

/**
 * @brief Return expansion board handle (pointer to the L6470 motor driver structure)
 * @param id Identifier inside the daisy chain
 * @retval Pointer to the StepperMotorDriverHandle_t structure
 */
StepperMotorBoardHandle_t *BSP_GetExpansionBoardHandle(uint8_t id)
{
  if (id == EXPBRD_ID(0))
    return (&XNUCLEOIHM02A1Drv[0]);
  if (id == EXPBRD_ID(1))
    return (&XNUCLEOIHM02A1Drv[1]);
  if (id == EXPBRD_ID(2))
    return (&XNUCLEOIHM02A1Drv[2]);
  if (id == EXPBRD_ID(3))
    return (&XNUCLEOIHM02A1Drv[3]);
  
  return 0;
}

/**
  * @brief  Configures each L6470 mounted on the X-NUCLEO-IHM02A1.
  * @param  StepperMotorBoardHandle_t* The pointer to the stepper motor board handle structure.
  * @param  MotorParameterData_t* The pointer to the parameter data structure.
  */
void BSP_Config(StepperMotorBoardHandle_t *StepperMotorBoardHandle, MotorParameterData_t *MotorParameterData)
{
  StepperMotorBoardHandle->Select(StepperMotorBoardHandle->StackedPosition);
  
  StepperMotorBoardHandle->StepperMotorDriverHandle[0]->Config(MotorParameterData+0);
  StepperMotorBoardHandle->StepperMotorDriverHandle[1]->Config(MotorParameterData+1);
}

/**
  * @}
  */ /* End of X-NUCLEO-IHM02A1_Exported_Functions */

/**
  * @}
  */ /* End of X-NUCLEO-IHM02A1 */

/**
  * @}
  */ /* End of BSP */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
