/**
  ******************************************************************************
  * @file       L6470.c
  * @date       01/10/2014 12:00:00
  * @brief      This file provides set of firmware functions to manage the
  *             L6470.
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
#include "L6470.h"
#include "xnucleoihm02a1_interface.h"

/**
  * @addtogroup BSP
  * @{
  */

/**
  * @addtogroup Components
  * @{
  */

/**
  * @addtogroup L6470
  * @{
  */
    
/**
  * @defgroup   L6470_Private_Constants
  * @brief      L6470 Private Constants.
  * @{
  */

/**
  * @brief Array whose elements are a structure in which store information about
  *        the L6470 Registers (the address, the names, the length in bits, the
  *        reset value)
  */
const sL6470_Register_t L6470_Register[L6470REGIDSIZE] = {
  {0x01, "ABS_POS",    22, 3, 0x000000},  //!< Current position
  {0x02, "EL_POS",      9, 2, 0x000},     //!< Electrical position
  {0x03 , "MARK",      22, 3, 0x000000},  //!< Mark position
  {0x04 , "SPEED",     20, 3, 0x0000},    //!< Current speed
  {0x05 , "ACC",       12, 2, 0x08A},     //!< Acceleration
  {0x06 , "DEC",       12, 2, 0x08A},     //!< Deceleration
  {0x07 , "MAX_SPEED", 10, 2, 0x041},     //!< Maximum speed
  {0x08 , "MIN_SPEED", 13, 2, 0x000},     //!< Minimum speed
  {0x15 , "FS_SPD",    10, 2, 0x027},     //!< Full-step speed
  {0x09 , "KVAL_HOLD",  8, 1, 0x29},      //!< Holding KVAL
  {0x0A , "KVAL_RUN",   8, 1, 0x29},      //!< Constant speed KVAL
  {0x0B , "KVAL_ACC",   8, 1, 0x29},      //!< Acceleration starting KVAL
  {0x0C , "KVAL_DEC",   8, 1, 0x29},      //!< Deceleration starting KVAL
  {0x0D , "INT_SPEED", 14, 2, 0x0408},    //!< Intersect speed
  {0x0E , "ST_SLP",     8, 1, 0x19},      //!< Start slope
  {0x0F , "FN_SLP_ACC", 8, 1, 0x29},      //!< Acceleration final slope
  {0x10 , "FN_SLP_DEC", 8, 1, 0x29},      //!< Deceleration final slope
  {0x11 , "K_THERM",    4, 1, 0x0},       //!< Thermal compensation factor
  {0x12 , "ADC_OUT",    5, 1, 0x00},      //!< ADC output, (the reset value is according to startup conditions)
  {0x13 , "OCD_TH",     4, 1, 0x8},       //!< OCD threshold
  {0x14 , "STALL_TH",   7, 1, 0x40},      //!< STALL threshold
  {0x16 , "STEP_MODE",  8, 1, 0x7},       //!< Step mode
  {0x17 , "ALARM_EN",   8, 1, 0xFF},      //!< Alarm enable
  {0x18 , "CONFIG",    16, 2, 0x2E88},    //!< IC configuration
  {0x19 , "STATUS",    16, 2, 0x0000}     //!< Status, (the reset value is according to startup conditions)
};

/**
  * @brief Array whose elements are a structure in which store information about
  *        the L6470 Application Commands (the mnemonic name, the number of
  *        needed parameters, the related funtion to call)
  */
const sL6470_ApplicationCommand_t L6470_ApplicationCommand[L6470APPCMDIDSIZE] =  {
  {"NOP",         0x00, 0},
  {"SETPARAM",    0x00, 2},
  {"GETPARAM",    0x20, 1},
  {"RUN",         0x50, 2},
  {"STEPCLOCK",   0x58, 1},
  {"MOVE",        0x40, 2},
  {"GOTO",        0x60, 1},
  {"GOTO_DIR",    0x68, 2},
  {"GOUNTIL",     0x82, 3},
  {"RELEASESW",   0x92, 2},
  {"GOHOME",      0x70, 0},
  {"GOMARK",      0x78, 0},
  {"RESETPOS",    0xD8, 0},
  {"RESETDEVICE", 0xC0, 0},
  {"SOFTSTOP",    0xB0, 0},
  {"HARDSTOP",    0xB8, 0},
  {"SOFTHIZ",     0xA0, 0},
  {"HARDHIZ",     0xA8, 0},
  {"GETSTATUS",   0xD0, 0}
};

/**
  * @brief The mnemonic names for the L6470 direction
  */
const sL6470_Direction_t L6470_Direction[L6470DIRIDSIZE] = {
  {"REV", 0x00},  //!< Reverse direction
  {"FWD", 0x01}   //!< Forward direction
};

/**
  * @brief Action taken about ABS_POS register
  */
const sL6470_ACT_t L6470_ACT[L6470ACTIDSIZE] = {
  {"RST", 0x00},  //!< ABS_POS register is reset
  {"CPY", 0x01}   //!< ABS_POS register value is copied into the MARK register
};

/**
  * @}
  */ /* End of L6470_Private_Constants */

/**
  * @defgroup   L6470_Private_Macros
  * @brief      L6470 Private Macros.
  * @{
  */
#define L6470_SPI_Communication     HAL_SPI_TransmitReceive   //!< Rename the HAL function to transmit and receive data via SPI
#define _DELAY(TDISCS)              for(delay_cnt=0; delay_cnt<20; delay_cnt++) __NOP()   //!<Simply deselect time delay for SPI nCS
/**
  * @}
  */ /* End of L6470_Private_Macros */

/**
  * @defgroup   L6470_Private_Variables
  * @brief      L6470 Private Variables.
  * @{
  */

/**
  * @brief      L6470 Application Command
  */
StepperMotorCommand_t L6470Command =
{
  L6470_SetParam,
  L6470_GetParam,
  L6470_Run,
  L6470_StepClock,
  L6470_Move,
  L6470_GoTo,
  L6470_GoToDir,
  L6470_GoUntil,
  L6470_ReleaseSW,
  L6470_GoHome,
  L6470_GoMark,
  L6470_ResetPos,
  L6470_ResetDevice,
  L6470_SoftStop,
  L6470_HardStop,
  L6470_SoftHiZ,
  L6470_HardHiZ,
  L6470_GetStatus,
  L6470_PrepareSetParam,
  L6470_PrepareGetParam,
  L6470_PrepareRun,
  L6470_PrepareStepClock,
  L6470_PrepareMove,
  L6470_PrepareGoTo,
  L6470_PrepareGoToDir,
  L6470_PrepareGoUntil,
  L6470_PrepareReleaseSW,
  L6470_PrepareGoHome,
  L6470_PrepareGoMark,
  L6470_PrepareResetPos,
  L6470_PrepareResetDevice,
  L6470_PrepareSoftStop,
  L6470_PrepareHardStop,
  L6470_PrepareSoftHiZ,
  L6470_PrepareHardHiZ,
  L6470_PrepareGetStatus,
  L6470_CheckStatusRegisterFlag,
};

sL6470_AppCmdPkg_t L6470_AppCmdPkg[L6470DAISYCHAINSIZE];                             //!< To store the identifier of the actual L6470 application command and its the needed parameters
uint8_t L6470_DaisyChainSpiTxStruct[L6470MAXSPICMDBYTESIZE][L6470DAISYCHAINSIZE];  //!< To store the matrix that contains the command data that are going to be sent by SPI to the L6470 daisy chain
uint8_t L6470_DaisyChainSpiRxStruct[L6470MAXSPICMDBYTESIZE][L6470DAISYCHAINSIZE];  //!< To store the matrix that contains the received data by SPI from the L6470 daisy chain
eFlagStatus_t L6470_DaisyChain_HalfPrepared = ZERO_F;                                     //!< Boolean variable used when more than one L6470 into the daisy chain is going to be addressed for commanding
sL6470_StatusRegister_t L6470_StatusRegister;                                          //!< To store the received L6470_StatusRegister
sL6470_StatusRegister_t *pL6470_StatusRegister = &L6470_StatusRegister;                //!< Pointer to the L6470_StatusRegister variable
uint8_t delay_cnt;                                                                    //!< To be used into simply deselect time delay for SPI nCS
/**
  * @}
  */ /* End of L6470_Private_Variables */

/* Function Prototypes -------------------------------------------------------*/

/**
  * @defgroup L6470_Private_Functions
  * @brief    L6470 Private Functions.
  * @{
  */

void L6470_ResetAppCmdPkg(sL6470_AppCmdPkg_t* pL6470_AppCmdPkg);
void L6470_FillAppCmdPkg(uint8_t L6470_Id, sL6470_AppCmdPkg_t* pL6470_AppCmdPkg, eL6470_AppCmdId_t L6470_AppCmdId, uint32_t p1, uint32_t p2, uint32_t p3);
void L6470_PrepareAppCmdPkg(uint8_t L6470_Id, sL6470_AppCmdPkg_t* pL6470_AppCmdPkg, eL6470_AppCmdId_t L6470_AppCmdId, uint32_t p1, uint32_t p2, uint32_t p3);
void L6470_PrepareDaisyChainCommand(sL6470_AppCmdPkg_t* pL6470_AppCmdPkg, uint8_t* pL6470_DaisyChainSpiTxStruct);

/**
  * @}
  */ /* End of L6470_Private_Functions */

/**
  * @addtogroup L6470_Private_Functions
  * @{
  */

/**
  * @brief  Reset the structure used to store the identifier of the L6470
  *         application command and its the needed parameters.
  * @param  L6470_AppCmdPkg   The structure to be reset.
  */
void L6470_ResetAppCmdPkg(sL6470_AppCmdPkg_t* pL6470_AppCmdPkg)
{
  uint8_t id;
  
  for(id=0; id<L6470DAISYCHAINSIZE; id++)
  {
    (pL6470_AppCmdPkg+id)->L6470_AppCmdId=(eL6470_AppCmdId_t)0;
    (pL6470_AppCmdPkg+id)->p1=0;
    (pL6470_AppCmdPkg+id)->p2=0;
    (pL6470_AppCmdPkg+id)->p3=0;
  }
}

/**
  * @brief  Fill the structure used to store the identifier of the L6470
  *         application command and its the needed parameters.
  * @param  L6470_Id          The identifier of the L6470 target inside the daisy chain.
  * @param  L6470_AppCmdPkg   The structure to be filled.
  * @param  L6470_AppCmdId    The identifier of the L6470 application command to be sent.
  * @param  p1                The 1st parameter (if it is not needed it will be not considered).
  * @param  p2                The 2nd parameter (if it is not needed it will be not considered).
  * @param  p3                The 3rd parameter (if it is not needed it will be not considered).
  */
void L6470_FillAppCmdPkg(uint8_t L6470_Id, sL6470_AppCmdPkg_t* pL6470_AppCmdPkg, eL6470_AppCmdId_t L6470_AppCmdId, uint32_t p1, uint32_t p2, uint32_t p3)
{
  (pL6470_AppCmdPkg+L6470_Id)->L6470_AppCmdId = L6470_AppCmdId;
  (pL6470_AppCmdPkg+L6470_Id)->p1 = p1;
  (pL6470_AppCmdPkg+L6470_Id)->p2 = p2;
  (pL6470_AppCmdPkg+L6470_Id)->p3 = p3;
}

/**
  * @brief  This function will fill the column of the L6470_AppCmdPkg related
  *         the L6470 to be addressed inside the daisy chain.
  *
  * @param  L6470_Id          The identifier of the L6470 target inside the daisy chain.
  * @param  pL6470_AppCmdPkg  Pointer to the sL6470_AppCmdPkg_t to be filled.
  * @param  L6470_AppCmdId    The identifier of the L6470 application command to be sent.
  * @param  p1                The 1st parameter (if it is not needed it will be not considered).
  * @param  p2                The 2nd parameter (if it is not needed it will be not considered).
  * @param  p3                The 3rd parameter (if it is not needed it will be not considered).
  */
void L6470_PrepareAppCmdPkg(uint8_t L6470_Id, sL6470_AppCmdPkg_t* pL6470_AppCmdPkg, eL6470_AppCmdId_t L6470_AppCmdId, uint32_t p1, uint32_t p2, uint32_t p3)
{
  if(!L6470_DaisyChain_HalfPrepared)
  {
    L6470_DaisyChain_HalfPrepared = ONE_F; /* To avoid to delete the previous entered command */
    L6470_ResetAppCmdPkg(pL6470_AppCmdPkg);
  }
  
  L6470_FillAppCmdPkg(L6470_Id, pL6470_AppCmdPkg, L6470_AppCmdId, p1, p2, p3);
}

/**
  * @brief  This function will translate the data inside the L6470_AppCmdPkg into
  *         the right data to be sent via SPI to the L6470 daisy chain.
  *
  * @param  pL6470_AppCmdPkg              Pointer to the sL6470_AppCmdPkg_t to be filled.
  * @param  pL6470_DaisyChainSpiTxStruct  Pointer to the structure used by SPI to send the commands.
  */
void L6470_PrepareDaisyChainCommand(sL6470_AppCmdPkg_t* pL6470_AppCmdPkg, uint8_t* pL6470_DaisyChainSpiTxStruct)
{
  uint8_t PkgId;
  uint8_t PARAMLengthBytes; /* The number of bytes related to the numeric value for the addressed register */
  uint8_t spibyte;
  uint8_t i;
  
  /* Reset the structure used to send the command to the L6470 Daisy Chain through the SPI */
  i = 0;
  for(spibyte=0;spibyte<L6470MAXSPICMDBYTESIZE;spibyte++)
    for(PkgId=0; PkgId<L6470DAISYCHAINSIZE; PkgId++)
      *(pL6470_DaisyChainSpiTxStruct+(i++)) = 0x00;
  
  for(PkgId=0; PkgId<L6470DAISYCHAINSIZE; PkgId++)
  {
    /* Build the 1st bytes to transmit with the binary code of the command */
    *(pL6470_DaisyChainSpiTxStruct+((0*L6470DAISYCHAINSIZE)+PkgId)) = (L6470_ApplicationCommand[(pL6470_AppCmdPkg+PkgId)->L6470_AppCmdId].BinaryCode);
    
    /* Perform the related L6470_AppCmdId */
    switch((pL6470_AppCmdPkg+PkgId)->L6470_AppCmdId)
    {
    case L6470_NOP_ID:
      break;
    case L6470_SETPARAM_ID:
      /* Build the 1st bytes to transmit (PARAM) */
      *(pL6470_DaisyChainSpiTxStruct+((0*L6470DAISYCHAINSIZE)+PkgId)) |= (L6470_Register[((pL6470_AppCmdPkg+PkgId)->p1)].Address);
      
      /* The length, in byte, of this register (PARAM) is... */
      PARAMLengthBytes = L6470_Register[((pL6470_AppCmdPkg+PkgId)->p1)].LengthByte;
      
      /* Build the others bytes to transmit (VALUE) */
      for (spibyte=1; spibyte<(PARAMLengthBytes+1); spibyte++)
      {
        *(pL6470_DaisyChainSpiTxStruct+((spibyte*L6470DAISYCHAINSIZE)+PkgId)) = (uint8_t)(((pL6470_AppCmdPkg+PkgId)->p2) >> (8*(PARAMLengthBytes-spibyte)));
      }
      break;
    case L6470_GETPARAM_ID:
      /* Build the 1st bytes to transmit (PARAM) */
      *(pL6470_DaisyChainSpiTxStruct+((0*L6470DAISYCHAINSIZE)+PkgId)) |= (L6470_Register[((pL6470_AppCmdPkg+PkgId)->p1)].Address);
      break;
    case L6470_RUN_ID:
      /* Build the 1st bytes to transmit (DIR) */
      *(pL6470_DaisyChainSpiTxStruct+((0*L6470DAISYCHAINSIZE)+PkgId)) |= (L6470_Direction[((pL6470_AppCmdPkg+PkgId)->p1)].BinaryCode);
      
      /* Build the others bytes to transmit (SPD) */
      for (spibyte=1; spibyte<(3+1); spibyte++)
      {
        *(pL6470_DaisyChainSpiTxStruct+((spibyte*L6470DAISYCHAINSIZE)+PkgId)) = (uint8_t)(((pL6470_AppCmdPkg+PkgId)->p2) >> (8*(3-spibyte)));
      }
      break;
    case L6470_STEPCLOCK_ID:
      /* Build the 1st bytes to transmit (DIR) */
      *(pL6470_DaisyChainSpiTxStruct+((0*L6470DAISYCHAINSIZE)+PkgId)) |= (L6470_Direction[((pL6470_AppCmdPkg+PkgId)->p1)].BinaryCode);
      break;
    case L6470_MOVE_ID:
      /* Build the 1st bytes to transmit (DIR) */
      *(pL6470_DaisyChainSpiTxStruct+((0*L6470DAISYCHAINSIZE)+PkgId)) |= (L6470_Direction[((pL6470_AppCmdPkg+PkgId)->p1)].BinaryCode);
      
      /* Build the others bytes to transmit (N_STEP) */
      for (spibyte=1; spibyte<(3+1); spibyte++)
      {
        *(pL6470_DaisyChainSpiTxStruct+((spibyte*L6470DAISYCHAINSIZE)+PkgId)) = (uint8_t)(((pL6470_AppCmdPkg+PkgId)->p2) >> (8*(3-spibyte)));
      }      
      break;
    case L6470_GOTO_ID:
      /* Build the others bytes to transmit (ABS_POS) */
      for (spibyte=1; spibyte<(3+1); spibyte++)
      {
        *(pL6470_DaisyChainSpiTxStruct+((spibyte*L6470DAISYCHAINSIZE)+PkgId)) = (uint8_t)(((pL6470_AppCmdPkg+PkgId)->p1) >> (8*(3-spibyte)));
      }
      break;
    case L6470_GOTODIR_ID:
      /* Build the 1st bytes to transmit (DIR) */
      *(pL6470_DaisyChainSpiTxStruct+((0*L6470DAISYCHAINSIZE)+PkgId)) |= (L6470_Direction[((pL6470_AppCmdPkg+PkgId)->p1)].BinaryCode);
      
      /* Build the others bytes to transmit (ABS_POS) */
      for (spibyte=1; spibyte<(3+1); spibyte++)
      {
        *(pL6470_DaisyChainSpiTxStruct+((spibyte*L6470DAISYCHAINSIZE)+PkgId)) = (uint8_t)(((pL6470_AppCmdPkg+PkgId)->p2) >> (8*(3-spibyte)));
      }
      break;
    case L6470_GOUNTIL_ID:
      /* Build the 1st bytes to transmit (ACT) */
      *(pL6470_DaisyChainSpiTxStruct+((0*L6470DAISYCHAINSIZE)+PkgId)) |= ((L6470_ACT[((pL6470_AppCmdPkg+PkgId)->p1)].BinaryCode)<<3);
      /* Build the 1st bytes to transmit (DIR) */
      *(pL6470_DaisyChainSpiTxStruct+((0*L6470DAISYCHAINSIZE)+PkgId)) |= (L6470_Direction[((pL6470_AppCmdPkg+PkgId)->p2)].BinaryCode);
      
      /* Build the others bytes to transmit (SPD) */
      for (spibyte=1; spibyte<(3+1); spibyte++)
      {
        *(pL6470_DaisyChainSpiTxStruct+((spibyte*L6470DAISYCHAINSIZE)+PkgId)) = (uint8_t)(((pL6470_AppCmdPkg+PkgId)->p3) >> (8*(3-spibyte)));
      }
      break;
    case L6470_RELEASESW_ID:
      /* Build the 1st bytes to transmit (ACT) */
      *(pL6470_DaisyChainSpiTxStruct+((0*L6470DAISYCHAINSIZE)+PkgId)) |= ((L6470_ACT[((pL6470_AppCmdPkg+PkgId)->p1)].BinaryCode)<<3);
      /* Build the 1st bytes to transmit (DIR) */
      *(pL6470_DaisyChainSpiTxStruct+((0*L6470DAISYCHAINSIZE)+PkgId)) |= (L6470_Direction[((pL6470_AppCmdPkg+PkgId)->p2)].BinaryCode);
      break;
    case L6470_GOHOME_ID:
      break;
    case L6470_GOMARK_ID:
      break;
    case L6470_RESETPOS_ID:
      break;
    case L6470_RESETDEVICE_ID:
      break;
    case L6470_SOFTSTOP_ID:
      break;
    case L6470_HARDSTOP_ID:
      break;
    case L6470_SOFTHIZ_ID:
      break;
    case L6470_HARDHIZ_ID:
      break;
    case L6470_GETSTATUS_ID:
      break;
    }
  }
}

/**
  * @}
  */ /* End of L6470_Private_Functions */

/**
  * @addtogroup L6470_Exported_Functions
  * @{
  */

/**
  * @addtogroup L6470_Conversion_Functions
  * @brief      The following functions act just on one driver inside the L6470
  *             daisy chain. The command is immediately sent.
  * @{
  */

 /**
  * @brief  Convert the absolute position as 2's complement format into the signed number.
  * 
  * @param  AbsPos      The absolute position in the range from [-(2^21)] to [+(2^21)-1].
  * @retval Position    The position as signed number.
  */
int32_t AbsPos_2_Position(uint32_t AbsPos)
{
  if (AbsPos > L6470_MAX_POSITION)
    return (AbsPos - (L6470_POSITION_RANGE + 1));
  else
    return AbsPos;
}

/**
  * @brief  Convert the position as signed number into absolute position as 2's complement format.
  * 
  * @param  Position    The position as signed number.
  * @retval AbsPos      The absolute position in the range from [-(2^21)] to [+(2^21)-1].
  */
uint32_t Position_2_AbsPos(int32_t Position)
{
  if ((Position >= 0) && (Position <= L6470_MAX_POSITION))
    return Position;
  else
  {
    if ((Position >= L6470_MIN_POSITION) && (Position < 0))
      return (Position + (L6470_POSITION_RANGE + 1));
    else
      return (L6470_POSITION_RANGE + 1);        // OVF
  }
}

/**
  * @brief  Convert the SPEED register value into step/s.
  * 
  * @param  Speed       The SPEED register value.
  * @retval step/s      The speed as step/s.
  */
float Speed_2_Step_s(uint32_t Speed)
{
  return (Speed * ((float)14.9012e-3));
}

/**
  * @brief  Convert the speed as step/s into a right value for SPEED register.
  * 
  * @param  step/s      The speed as step/s.
  * @retval Speed       The SPEED register value.
  */
uint32_t Step_s_2_Speed(float Step_s)
{
  if (Step_s <= (L6470_MAX_SPEED * ((float)14.9012e-3)))
    return (uint32_t)(Step_s / ((float)14.9012e-3));
  else
    return 0;   // Warning
}

/**
  * @brief  Convert the ACC register value into step/(s^2).
  * 
  * @param  Acc         The ACC register value.
  * @retval step/(s^2)  The acceleration as step/(s^2).
  */
float Acc_2_Step_s2(uint16_t Acc)
{
  if (Acc <= L6470_MAX_ACC)
    return (Acc * ((float)1.4552e1));
  else
    return 0;   // Warning
}

/**
  * @brief  Convert the acceleartion as step/(s^2) into a right value for ACC register.
  * 
  * @param  step/(s^2)  The acceleration as step/(s^2).
  * @retval Acc         The ACC register value.
  */
uint16_t Step_s2_2_Acc(float Step_s2)
{
  if (Step_s2 <= (L6470_MAX_ACC * ((float)1.4552e1)))
    return (uint16_t)(Step_s2 / ((float)1.4552e1));
  else
    return 0;   // Warning
}

/**
  * @brief  Convert the DEC register value into step/(s^2).
  * 
  * @param  Dec         The DEC register value.
  * @retval step/(s^2)  The deceleration as step/(s^2).
  */
float Dec_2_Step_s2(uint16_t Dec)
{
  if (Dec <= L6470_MAX_DEC)
    return (Dec * ((float)1.4552e1));
  else
    return 0;   // Warning
}

/**
  * @brief  Convert the deceleration as step/(s^2) into a right value for DEC register.
  * 
  * @param  step/(s^2)  The deceleration as step/(s^2).
  * @retval Dec         The DEC register value.
  */
uint16_t Step_s2_2_Dec(float Step_s2)
{
  if (Step_s2 <= (L6470_MAX_DEC * ((float)1.4552e1)))
    return (uint16_t)(Step_s2 / ((float)1.4552e1));
  else
    return 0;   // Warning
}

/**
  * @brief  Convert the MAX_SPEED register value into step/s.
  * 
  * @param  MaxSpeed    The MAX_SPEED register value.
  * @retval step/s      The max speed as step/s.
  */
float MaxSpeed_2_Step_s(uint16_t MaxSpeed)
{
  if (MaxSpeed <= L6470_MAX_MAX_SPEED)
    return (MaxSpeed * ((float)15.2588));
  else
    return 0;   // Warning
}

/**
  * @brief  Convert the max speed as step/s into a right value for MAX_SPEED register.
  * 
  * @param  step/s      The max speed as step/s.
  * @retval MaxSpeed    The MAX_SPEED register value.
  */
uint16_t Step_s_2_MaxSpeed(float Step_s)
{
  if (Step_s <= (L6470_MAX_MAX_SPEED * ((float)15.2588)))
    return (uint16_t)(Step_s / ((float)15.2588));
  else
    return 0;   // Warning
}

/**
  * @brief  Convert the MIN_SPEED register value into step/s.
  * 
  * @param  MinSpeed    The MIN_SPEED register value.
  * @retval step/s      The min speed as step/s.
  */
float MinSpeed_2_Step_s(uint16_t MinSpeed)
{
  if (MinSpeed <= L6470_MAX_MIN_SPEED)
    return (MinSpeed * ((float)238.4186e-3));
  else
    return 0;   // Warning
}

/**
  * @brief  Convert the min speed as step/s into a right value for MIN_SPEED register.
  * 
  * @param  step/s      The min speed as step/s.
  * @retval MinSpeed    The MIN_SPEED register value.
  */
uint16_t Step_s_2_MinSpeed(float Step_s)
{
  if (Step_s <= (L6470_MAX_MIN_SPEED * ((float)238.4186e-3)))
    return (uint16_t)(Step_s / ((float)238.4186e-3));
  else
    return 0;   // Warning
}

/**
  * @brief  Convert the FS_SPD register value into step/s.
  * 
  * @param  FsSpd       The FS_SPD register value.
  * @retval step/s      The full-step speed as step/s.
  */
float FsSpd_2_Step_s(uint16_t FsSpd)
{
  if (FsSpd <= L6470_MAX_FS_SPD)
    return ((FsSpd+0.5) * ((float)15.25));
  else
    return 0;   // Warning
}

/**
  * @brief  Convert the full-step speed as step/s into a right value for FS_SPD register.
  * 
  * @param  step/s      The full-step speed as step/s.
  * @retval FsSpd       The FS_SPD register value.
  */
uint16_t Step_s_2_FsSpd(float Step_s)
{
  if (Step_s <= ((L6470_MAX_FS_SPD+0.5) * ((float)15.25)))
    return (uint16_t)((float)(Step_s / ((float)15.25)) - (float)0.5);
  else
    return 0;   // Warning
}

/**
  * @brief  Convert the INT_SPEED register value into step/s.
  * 
  * @param  IntSpeed    The INT_SPEED register value.
  * @retval step/s      The intersect speed as step/s.
  */
float IntSpeed_2_Step_s(uint16_t IntSpeed)
{
  if (IntSpeed <= L6470_MAX_INT_SPEED)
    return (IntSpeed * ((float)59.6046e-3));
  else
    return 0;   // Warning
}

/**
  * @brief  Convert the intersect speed as step/s into a right value for INT_SPEED register.
  * 
  * @param  step/s      The full-step speed as step/s.
  * @retval FsSpd       The FS_SPD register value.
  */
uint16_t Step_s_2_IntSpeed(float Step_s)
{
  if (Step_s <= (L6470_MAX_INT_SPEED * ((float)59.6046e-3)))
    return (uint16_t)(Step_s / ((float)59.6046e-3));
  else
    return 0;   // Warning
}

/**
  * @brief  Convert the ST_SLP register value into s/step.
  * 
  * @param  StartSlope  The ST_SLP register value.
  * @retval s/step      The start slope as s/step.
  */
float StSlp_2_s_Step(uint8_t StSlp)
{
//  if (StSlp <= L6470_MAX_ST_SLP)
    return (StSlp * ((float)1.5686e-5));
//  else
//    return 0;   // Warning
}

/**
  * @brief  Convert the intersect speed as step/s into a right value for INT_SPEED register.
  * 
  * @param  step/s      The full-step speed as step/s.
  * @retval FsSpd       The FS_SPD register value.
  */
uint8_t s_Step_2_StSlp(float s_Step)
{
  if (s_Step <= (L6470_MAX_ST_SLP * ((float)1.5686e-5)))
    return (uint8_t)(s_Step / ((float)1.5686e-5));
  else
    return 0;   // Warning
}

/**
  * @brief  Convert the INT_SPEED register value into step/s.
  * 
  * @param  IntSpeed    The INT_SPEED register value.
  * @retval step/s      The intersect speed as step/s.
  */
float FnSlpAcc_2_s_Step(uint8_t FnSlpAcc)
{
//  if (FnSlpAcc <= L6470_MAX_FN_SLP_ACC)
    return (FnSlpAcc * ((float)1.5686e-5));
//  else
//    return 0;   // Warning
}

/**
  * @brief  Convert the intersect speed as step/s into a right value for INT_SPEED register.
  * 
  * @param  step/s      The full-step speed as step/s.
  * @retval FsSpd       The FS_SPD register value.
  */
uint8_t s_Step_2_FnSlpAcc(float s_Step)
{
  if (s_Step <= (L6470_MAX_FN_SLP_ACC * ((float)1.5686e-5)))
    return (uint8_t)(s_Step / ((float)1.5686e-5));
  else
    return 0;   // Warning
}

/**
  * @brief  Convert the INT_SPEED register value into step/s.
  * 
  * @param  IntSpeed    The INT_SPEED register value.
  * @retval step/s      The intersect speed as step/s.
  */
float FnSlpDec_2_s_Step(uint8_t FnSlpDec)
{
//  if (FnSlpDec <= L6470_MAX_FN_SLP_DEC)
    return (FnSlpDec * ((float)1.5686e-5));
//  else
//    return 0;   // Warning
}

/**
  * @brief  Convert the intersect speed as step/s into a right value for INT_SPEED register.
  * 
  * @param  step/s      The full-step speed as step/s.
  * @retval FsSpd       The FS_SPD register value.
  */
uint8_t s_Step_2_FnSlpDec(float s_Step)
{
  if (s_Step <= (L6470_MAX_FN_SLP_DEC * ((float)1.5686e-5)))
    return (uint8_t)(s_Step / ((float)1.5686e-5));
  else
    return 0;   // Warning
}

/**
  * @brief  Convert the OCD_TH register value into mA.
  * 
  * @param  OcdTh       The OCD_TH register value.
  * @retval mA          The overcurrent threshold as mA.
  */
float OcdTh_2_mA(uint8_t OcdTh)
{
  if (OcdTh <= L6470_MAX_OCD_TH)
    return ((OcdTh+1) * ((float)375));
  else
    return 0;   // Warning
}

/**
  * @brief  Convert the overcurrent threshold as mA into a right value for OCD_TH register.
  * 
  * @param  mA          The overcurrent threshold as mA.
  * @retval OcdTh       The OCD_TH register value.
  */
uint8_t mA_2_OcdTh(float mA)
{
  float result, decimal;
  
  if (mA <= ((L6470_MAX_OCD_TH+1) * ((float)375)))
  {
    result = (mA / ((float)375));
    decimal = result - (uint8_t)result;
    
    if (decimal < (float)0.5)
      return ((uint8_t)result - 1);
    else
      return ((uint8_t)result);    
  }
  else
    return 0;   // Warning
}

/**
  * @brief  Convert the STALL_TH register value into mA.
  * 
  * @param  StallTh     The STALL_TH register value.
  * @retval mA          The stall detection threshold as mA.
  */
float StallTh_2_mA(uint8_t StallTh)
{
  if (StallTh <= L6470_MAX_STALL_TH)
    return ((StallTh+1) * ((float)31.25));
  else
    return 0;   // Warning
}

/**
  * @brief  Convert the stall detection threshold as mA into a right value for STALL_TH register.
  * 
  * @param  mA          The stall detection threshold as mA.
  * @retval StallTh     The STALL_TH register value.
  */
uint8_t mA_2_StallTh(float mA)
{
  float result, decimal;
  
  if (mA <= ((L6470_MAX_STALL_TH+1) * ((float)31.25)))
  {
    result = (mA / ((float)31.25));
    decimal = result - (uint8_t)result;
    
    if (decimal < (float)0.5)
      return ((uint8_t)result - 1);
    else
      return ((uint8_t)result);    
  }
  else
    return 0;   // Warning
}

/**
  * @}
  */ /* End of L6470_Conversion_Functions */

/**
  * @addtogroup L6470_AppCMDs
  * @{
  */

/**
  * @brief  SetParam command sets the register value equal to a new value.
  * 
  * @param  L6470_Id      The identifier of the L6470 target inside the daisy chain.
  * @param  L6470_RegId   The identifier of the L6470 register to be addressed.
  * @param  Value         The new value.
  */
void L6470_SetParam(uint8_t L6470_Id, eL6470_RegId_t L6470_RegId, uint32_t Value)
{
  L6470_PrepareAppCmdPkg(L6470_Id, L6470_AppCmdPkg, L6470_SETPARAM_ID, L6470_RegId, Value, 0);
  L6470_PrepareDaisyChainCommand(L6470_AppCmdPkg, (uint8_t*)L6470_DaisyChainSpiTxStruct);
  L6470_DaisyChainCommand((uint8_t*)L6470_DaisyChainSpiTxStruct, (uint8_t*)L6470_DaisyChainSpiRxStruct);
}

/**
  * @brief  GetParam command reads the register value.
  * 
  * @param  L6470_Id      The identifier of the L6470 target inside the daisy chain.
  * @param  L6470_RegId   The identifier of the L6470 register to be addressed.
  * 
  * @retval ReceivedValue The register value.
  */
uint32_t L6470_GetParam(uint8_t L6470_Id, eL6470_RegId_t L6470_RegId)
{
  uint8_t ValueLengthByte;
  uint32_t ReceivedValue;

  L6470_PrepareAppCmdPkg(L6470_Id, L6470_AppCmdPkg, L6470_GETPARAM_ID, L6470_RegId, 0, 0);
  L6470_PrepareDaisyChainCommand(L6470_AppCmdPkg, (uint8_t*)L6470_DaisyChainSpiTxStruct);
  L6470_DaisyChainCommand((uint8_t*)L6470_DaisyChainSpiTxStruct, (uint8_t*)L6470_DaisyChainSpiRxStruct);
  
  ValueLengthByte = L6470_Register[L6470_RegId].LengthByte;
  
  ReceivedValue = L6470_ExtractReturnedData(L6470_Id, (uint8_t*)L6470_DaisyChainSpiRxStruct, ValueLengthByte);
  
  return ReceivedValue;
}

/**
  * @brief  Run command produces a motion at fixed speed.
  * 
  * @param  L6470_Id      The identifier of the L6470 target inside the daisy chain.
  * @param  L6470_DirId   The identifier of the L6470 motion direction.
  * @param  Speed         The speed value as (([step/s] * 250e-9) / 2^-28)
  */
void L6470_Run(uint8_t L6470_Id, eL6470_DirId_t L6470_DirId, uint32_t Speed)
{
  L6470_PrepareAppCmdPkg(L6470_Id, L6470_AppCmdPkg, L6470_RUN_ID, L6470_DirId, Speed, 0);
  L6470_PrepareDaisyChainCommand(L6470_AppCmdPkg, (uint8_t*)L6470_DaisyChainSpiTxStruct);
  L6470_DaisyChainCommand((uint8_t*)L6470_DaisyChainSpiTxStruct, (uint8_t*)L6470_DaisyChainSpiRxStruct);
}

/**
  * @brief  StepClock command switches the device in Step-clock mode.
  * 
  * @param  L6470_Id      The identifier of the L6470 target inside the daisy chain.
  * @param  L6470_DirId   The identifier of the L6470 motion direction.
  */
void L6470_StepClock(uint8_t L6470_Id, eL6470_DirId_t L6470_DirId)
{
  L6470_PrepareAppCmdPkg(L6470_Id, L6470_AppCmdPkg, L6470_STEPCLOCK_ID, L6470_DirId, 0, 0);
  L6470_PrepareDaisyChainCommand(L6470_AppCmdPkg, (uint8_t*)L6470_DaisyChainSpiTxStruct);
  L6470_DaisyChainCommand((uint8_t*)L6470_DaisyChainSpiTxStruct, (uint8_t*)L6470_DaisyChainSpiRxStruct);
}

/**
  * @brief  Move command produces a motion of N_STEP microsteps.
  * 
  * @param  L6470_Id      The identifier of the L6470 target inside the daisy chain.
  * @param  L6470_DirId   The identifier of the L6470 motion direction.
  * @param  N_Step        The number of microsteps.
  */
void L6470_Move(uint8_t L6470_Id, eL6470_DirId_t L6470_DirId, uint32_t N_Step)
{
  L6470_PrepareAppCmdPkg(L6470_Id, L6470_AppCmdPkg, L6470_MOVE_ID, L6470_DirId, N_Step, 0);
  L6470_PrepareDaisyChainCommand(L6470_AppCmdPkg, (uint8_t*)L6470_DaisyChainSpiTxStruct);
  L6470_DaisyChainCommand((uint8_t*)L6470_DaisyChainSpiTxStruct, (uint8_t*)L6470_DaisyChainSpiRxStruct);
}

/**
  * @brief  GoTo command produces a motion to ABS_POS absolute position through the shortest path.
  * 
  * @param  L6470_Id      The identifier of the L6470 target inside the daisy chain.
  * @param  AbsPos        The target absolute position.
  */
void L6470_GoTo(uint8_t L6470_Id, uint32_t AbsPos)
{
  if (AbsPos <= L6470_POSITION_RANGE)
  {
    L6470_PrepareAppCmdPkg(L6470_Id, L6470_AppCmdPkg, L6470_GOTO_ID, AbsPos, 0, 0);
    L6470_PrepareDaisyChainCommand(L6470_AppCmdPkg, (uint8_t*)L6470_DaisyChainSpiTxStruct);
    L6470_DaisyChainCommand((uint8_t*)L6470_DaisyChainSpiTxStruct, (uint8_t*)L6470_DaisyChainSpiRxStruct);
  }
}

/**
  * @brief  GoTo_DIR command produces a motion to ABS_POS absolute position imposing a direction.
  * 
  * @param  L6470_Id      The identifier of the L6470 target inside the daisy chain.
  * @param  L6470_DirId   The identifier of the L6470 motion direction.
  * @param  AbsPos        The target absolute position.
  */
void L6470_GoToDir(uint8_t L6470_Id, eL6470_DirId_t L6470_DirId, uint32_t AbsPos)
{
  if (AbsPos <= L6470_POSITION_RANGE)
  {
    L6470_PrepareAppCmdPkg(L6470_Id, L6470_AppCmdPkg, L6470_GOTODIR_ID, L6470_DirId, AbsPos, 0);
    L6470_PrepareDaisyChainCommand(L6470_AppCmdPkg, (uint8_t*)L6470_DaisyChainSpiTxStruct);
    L6470_DaisyChainCommand((uint8_t*)L6470_DaisyChainSpiTxStruct, (uint8_t*)L6470_DaisyChainSpiRxStruct);
  }
}

/**
  * @brief  GoUntil command produces a motion at fixed speed imposing a direction
  *         until an external switch turn-on event occurs.
  * 
  * @param  L6470_Id      The identifier of the L6470 target inside the daisy chain.
  * @param  L6470_ActId   The identifier of the L6470 action about the absolute position.
  * @param  L6470_DirId   The identifier of the L6470 motion direction.
  * @param  Speed         The speed value as (([step/s] * 250e-9) / 2^-28)
  */
void L6470_GoUntil(uint8_t L6470_Id, eL6470_ActId_t L6470_ActId, eL6470_DirId_t L6470_DirId, uint32_t Speed)
{
  L6470_PrepareAppCmdPkg(L6470_Id, L6470_AppCmdPkg, L6470_GOUNTIL_ID, L6470_ActId, L6470_DirId, Speed);
  L6470_PrepareDaisyChainCommand(L6470_AppCmdPkg, (uint8_t*)L6470_DaisyChainSpiTxStruct);
  L6470_DaisyChainCommand((uint8_t*)L6470_DaisyChainSpiTxStruct, (uint8_t*)L6470_DaisyChainSpiRxStruct);
}

/**
  * @brief  ReleaseSW command produces a motion at minimum speed imposing a direction
  *         until SW is released.
  * 
  * @param  L6470_Id      The identifier of the L6470 target inside the daisy chain.
  * @param  L6470_ActId   The identifier of the L6470 action about the absolute position.
  * @param  L6470_DirId   The identifier of the L6470 motion direction.
  */
void L6470_ReleaseSW(uint8_t L6470_Id, eL6470_ActId_t L6470_ActId, eL6470_DirId_t L6470_DirId)
{
  L6470_PrepareAppCmdPkg(L6470_Id, L6470_AppCmdPkg, L6470_RELEASESW_ID, L6470_ActId, L6470_DirId, 0);
  L6470_PrepareDaisyChainCommand(L6470_AppCmdPkg, (uint8_t*)L6470_DaisyChainSpiTxStruct);
  L6470_DaisyChainCommand((uint8_t*)L6470_DaisyChainSpiTxStruct, (uint8_t*)L6470_DaisyChainSpiRxStruct);
}

/**
  * @brief  GoHome command produces a motion to the HOME position (zero position)
  *         via the shortest path.
  * 
  * @param  L6470_Id      The identifier of the L6470 target inside the daisy chain.
  */
void L6470_GoHome(uint8_t L6470_Id)
{
  L6470_PrepareAppCmdPkg(L6470_Id, L6470_AppCmdPkg, L6470_GOHOME_ID, 0, 0, 0);
  L6470_PrepareDaisyChainCommand(L6470_AppCmdPkg, (uint8_t*)L6470_DaisyChainSpiTxStruct);
  L6470_DaisyChainCommand((uint8_t*)L6470_DaisyChainSpiTxStruct, (uint8_t*)L6470_DaisyChainSpiRxStruct);
}

/**
  * @brief  GoMark command produces a motion to the MARK position performing the
  *         minimum path.
  * 
  * @param  L6470_Id      The identifier of the L6470 target inside the daisy chain.
  */
void L6470_GoMark(uint8_t L6470_Id)
{
  L6470_PrepareAppCmdPkg(L6470_Id, L6470_AppCmdPkg, L6470_GOMARK_ID, 0, 0, 0);
  L6470_PrepareDaisyChainCommand(L6470_AppCmdPkg, (uint8_t*)L6470_DaisyChainSpiTxStruct);
  L6470_DaisyChainCommand((uint8_t*)L6470_DaisyChainSpiTxStruct, (uint8_t*)L6470_DaisyChainSpiRxStruct);
}

/**
  * @brief  ResetPos command resets the ABS_POS register to zero.
  * 
  * @param  L6470_Id      The identifier of the L6470 target inside the daisy chain.
  */
void L6470_ResetPos(uint8_t L6470_Id)
{
  L6470_PrepareAppCmdPkg(L6470_Id, L6470_AppCmdPkg, L6470_RESETPOS_ID, 0, 0, 0);
  L6470_PrepareDaisyChainCommand(L6470_AppCmdPkg, (uint8_t*)L6470_DaisyChainSpiTxStruct);
  L6470_DaisyChainCommand((uint8_t*)L6470_DaisyChainSpiTxStruct, (uint8_t*)L6470_DaisyChainSpiRxStruct);
}

/**
  * @brief  ResetDevice command resets the device to power-up conditions.
  * 
  * @param  L6470_Id      The identifier of the L6470 target inside the daisy chain.
  */
void L6470_ResetDevice(uint8_t L6470_Id)
{
  L6470_PrepareAppCmdPkg(L6470_Id, L6470_AppCmdPkg, L6470_RESETDEVICE_ID, 0, 0, 0);
  L6470_PrepareDaisyChainCommand(L6470_AppCmdPkg, (uint8_t*)L6470_DaisyChainSpiTxStruct);
  L6470_DaisyChainCommand((uint8_t*)L6470_DaisyChainSpiTxStruct, (uint8_t*)L6470_DaisyChainSpiRxStruct);
}

/**
  * @brief  SoftStop command causes an immediate deceleration to zero speed and
  *         a consequent motor stop; the deceleration value used is the one stored
  *         in the DEC register.
  * 
  * @param  L6470_Id      The identifier of the L6470 target inside the daisy chain.
  */
void L6470_SoftStop(uint8_t L6470_Id)
{
  L6470_PrepareAppCmdPkg(L6470_Id, L6470_AppCmdPkg, L6470_SOFTSTOP_ID, 0, 0, 0);
  L6470_PrepareDaisyChainCommand(L6470_AppCmdPkg, (uint8_t*)L6470_DaisyChainSpiTxStruct);
  L6470_DaisyChainCommand((uint8_t*)L6470_DaisyChainSpiTxStruct, (uint8_t*)L6470_DaisyChainSpiRxStruct);
}

/**
  * @brief  HardStop command causes an immediate motor stop with infinite deceleration.
  * 
  * @param  L6470_Id      The identifier of the L6470 target inside the daisy chain.
  */
void L6470_HardStop(uint8_t L6470_Id)
{
  L6470_PrepareAppCmdPkg(L6470_Id, L6470_AppCmdPkg, L6470_HARDSTOP_ID, 0, 0, 0);
  L6470_PrepareDaisyChainCommand(L6470_AppCmdPkg, (uint8_t*)L6470_DaisyChainSpiTxStruct);
  L6470_DaisyChainCommand((uint8_t*)L6470_DaisyChainSpiTxStruct, (uint8_t*)L6470_DaisyChainSpiRxStruct);
}

/**
  * @brief  SoftHiZ command disables the power bridges (high impedance state)
  *         after a deceleration to zero; the deceleration value used is the one
  *         stored in the DEC register.
  * 
  * @param  L6470_Id      The identifier of the L6470 target inside the daisy chain.
  */
void L6470_SoftHiZ(uint8_t L6470_Id)
{
  L6470_PrepareAppCmdPkg(L6470_Id, L6470_AppCmdPkg, L6470_SOFTHIZ_ID, 0, 0, 0);
  L6470_PrepareDaisyChainCommand(L6470_AppCmdPkg, (uint8_t*)L6470_DaisyChainSpiTxStruct);
  L6470_DaisyChainCommand((uint8_t*)L6470_DaisyChainSpiTxStruct, (uint8_t*)L6470_DaisyChainSpiRxStruct);
}

/**
  * @brief  HardHiZ command immediately disables the power bridges (high impedance state).
  * 
  * @param  L6470_Id      The identifier of the L6470 target inside the daisy chain.
  */
void L6470_HardHiZ(uint8_t L6470_Id)
{
  L6470_PrepareAppCmdPkg(L6470_Id, L6470_AppCmdPkg, L6470_HARDHIZ_ID, 0, 0, 0);
  L6470_PrepareDaisyChainCommand(L6470_AppCmdPkg, (uint8_t*)L6470_DaisyChainSpiTxStruct);
  L6470_DaisyChainCommand((uint8_t*)L6470_DaisyChainSpiTxStruct, (uint8_t*)L6470_DaisyChainSpiRxStruct);
}

/**
  * @brief  GetStatus command returns the STATUS register value.
  * 
  * @param  L6470_Id      The identifier of the L6470 target inside the daisy chain.
  * 
  * @retval ReceivedValue The register value.
  */
uint16_t L6470_GetStatus(uint8_t L6470_Id)
{
  uint16_t ReceivedValue;

  L6470_PrepareAppCmdPkg(L6470_Id, L6470_AppCmdPkg, L6470_GETSTATUS_ID, 0, 0, 0);
  L6470_PrepareDaisyChainCommand(L6470_AppCmdPkg, (uint8_t*)L6470_DaisyChainSpiTxStruct);
  L6470_DaisyChainCommand((uint8_t*)L6470_DaisyChainSpiTxStruct, (uint8_t*)L6470_DaisyChainSpiRxStruct);
  
  ReceivedValue = (uint16_t)L6470_ExtractReturnedData(L6470_Id, (uint8_t*)L6470_DaisyChainSpiRxStruct, 2);
  
  return ReceivedValue;
}

/**
  * @}
  */ /* End of L6470_AppCMDs */

/**
  * @addtogroup L6470_AppCMDs_ToBePrepared
  * @{
  */

/**
  * @brief  Prepare to send @ref L6470_SetParam command.
  *
  * @param  L6470_Id      The identifier of the L6470 target inside the daisy chain.
  * @param  L6470_RegId   The identifier of the L6470 register to be addressed.
  * @param  Value         The new value.
  *
  * @note   This function will properly fill the right column of the L6470_AppCmdPkg.
  * @note   The commad will be sent by @ref L6470_PerformPreparedApplicationCommand.
  */
void L6470_PrepareSetParam(uint8_t L6470_Id, eL6470_RegId_t L6470_RegId, uint32_t Value)
{
    L6470_PrepareAppCmdPkg(L6470_Id, L6470_AppCmdPkg, L6470_SETPARAM_ID, L6470_RegId, Value, 0);
}

/**
  * @brief  Prepare to send @ref L6470_GetParam command.
  *
  * @param  L6470_Id      The L6470 identifier inside the daisy chain.
  * @param  L6470_RegId   The identifier of the L6470 register to be addressed.
  *
  * @retval ReceivedValue The register value.
  *
  * @note   This function will properly fill the right column of the L6470_AppCmdPkg.
  * @note   The commad will be sent by @ref L6470_PerformPreparedApplicationCommand.
  */
void L6470_PrepareGetParam(uint8_t L6470_Id, eL6470_RegId_t L6470_RegId)
{
  L6470_PrepareAppCmdPkg(L6470_Id, L6470_AppCmdPkg, L6470_GETPARAM_ID, L6470_RegId, 0, 0);
}

/**
  * @brief  Prepare to send @ref L6470_Run command.
  * 
  * @param  L6470_Id      The identifier of the L6470 target inside the daisy chain.
  * @param  L6470_DirId   The identifier of the L6470 motion direction.
  * @param  Speed         The speed value as (([step/s] * 250e-9) / 2^-28)
  *
  * @note   This function will properly fill the right column of the L6470_AppCmdPkg.
  * @note   The commad will be sent by @ref L6470_PerformPreparedApplicationCommand.
  */
void L6470_PrepareRun(uint8_t L6470_Id, eL6470_DirId_t L6470_DirId, uint32_t Speed)
{
  L6470_PrepareAppCmdPkg(L6470_Id, L6470_AppCmdPkg, L6470_RUN_ID, L6470_DirId, Speed, 0);
}

/**
  * @brief  Prepare to send @ref L6470_StepClock command.
  * 
  * @param  L6470_Id      The identifier of the L6470 target inside the daisy chain.
  * @param  L6470_DirId   The identifier of the L6470 motion direction.
  *
  * @note   This function will properly fill the right column of the L6470_AppCmdPkg.
  * @note   The commad will be sent by @ref L6470_PerformPreparedApplicationCommand.
  */
void L6470_PrepareStepClock(uint8_t L6470_Id, eL6470_DirId_t L6470_DirId)
{
  L6470_PrepareAppCmdPkg(L6470_Id, L6470_AppCmdPkg, L6470_STEPCLOCK_ID, L6470_DirId, 0, 0);
}

/**
  * @brief  Prepare to send @ref L6470_Move command.
  * 
  * @param  L6470_Id      The identifier of the L6470 target inside the daisy chain.
  * @param  L6470_DirId   The identifier of the L6470 motion direction.
  * @param  N_Step        The number of microsteps.
  *
  * @note   This function will properly fill the right column of the L6470_AppCmdPkg.
  * @note   The commad will be sent by @ref L6470_PerformPreparedApplicationCommand.
  */
void L6470_PrepareMove(uint8_t L6470_Id, eL6470_DirId_t L6470_DirId, uint32_t N_Step)
{
  L6470_PrepareAppCmdPkg(L6470_Id, L6470_AppCmdPkg, L6470_MOVE_ID, L6470_DirId, N_Step, 0);
}

/**
  * @brief  Prepare to send @ref L6470_GoTo command.
  * 
  * @param  L6470_Id      The identifier of the L6470 target inside the daisy chain.
  * @param  AbsPos        The target absolute position.
  *
  * @note   This function will properly fill the right column of the L6470_AppCmdPkg.
  * @note   The commad will be sent by @ref L6470_PerformPreparedApplicationCommand.
  */
void L6470_PrepareGoTo(uint8_t L6470_Id, uint32_t AbsPos)
{
  L6470_PrepareAppCmdPkg(L6470_Id, L6470_AppCmdPkg, L6470_GOTO_ID, AbsPos, 0, 0);
}

/**
  * @brief  Prepare to send @ref L6470_GoToDIR command.
  * 
  * @param  L6470_Id      The identifier of the L6470 target inside the daisy chain.
  * @param  L6470_DirId   The identifier of the L6470 motion direction.
  * @param  AbsPos        The target absolute position.
  *
  * @note   This function will properly fill the right column of the L6470_AppCmdPkg.
  * @note   The commad will be sent by @ref L6470_PerformPreparedApplicationCommand.
  */
void L6470_PrepareGoToDir(uint8_t L6470_Id, eL6470_DirId_t L6470_DirId, uint32_t AbsPos)
{
  L6470_PrepareAppCmdPkg(L6470_Id, L6470_AppCmdPkg, L6470_GOTODIR_ID, L6470_DirId, AbsPos, 0);
}

/**
  * @brief  Prepare to send @ref L6470_GoUntil command.
  * 
  * @param  L6470_Id      The identifier of the L6470 target inside the daisy chain.
  * @param  L6470_ActId   The identifier of the L6470 action about the absolute position.
  * @param  L6470_DirId   The identifier of the L6470 motion direction.
  * @param  Speed         The speed value as (([step/s] * 250e-9) / 2^-28)
  *
  * @note   This function will properly fill the right column of the L6470_AppCmdPkg.
  * @note   The commad will be sent by @ref L6470_PerformPreparedApplicationCommand.
  */
void L6470_PrepareGoUntil(uint8_t L6470_Id, eL6470_ActId_t L6470_ActId, eL6470_DirId_t L6470_DirId, uint32_t Speed)
{
  L6470_PrepareAppCmdPkg(L6470_Id, L6470_AppCmdPkg, L6470_GOUNTIL_ID, L6470_ActId, L6470_DirId, Speed);
}

/**
  * @brief  Prepare to send @ref L6470_ReleaseSW.
  * 
  * @param  L6470_Id      The identifier of the L6470 target inside the daisy chain.
  * @param  L6470_ActId   The identifier of the L6470 action about the absolute position.
  * @param  L6470_DirId   The identifier of the L6470 motion direction.
  *
  * @note   This function will properly fill the right column of the L6470_AppCmdPkg.
  * @note   The commad will be sent by @ref L6470_PerformPreparedApplicationCommand.
  */
void L6470_PrepareReleaseSW(uint8_t L6470_Id, eL6470_ActId_t L6470_ActId, eL6470_DirId_t L6470_DirId)
{
  L6470_PrepareAppCmdPkg(L6470_Id, L6470_AppCmdPkg, L6470_RELEASESW_ID, L6470_ActId, L6470_DirId, 0);
}

/**
  * @brief  Prepare to send @ref L6470_GoHome command.
  * 
  * @param  L6470_Id      The identifier of the L6470 target inside the daisy chain.
  *
  * @note   This function will properly fill the right column of the L6470_AppCmdPkg.
  * @note   The commad will be sent by @ref L6470_PerformPreparedApplicationCommand.
  */
void L6470_PrepareGoHome(uint8_t L6470_Id)
{
  L6470_PrepareAppCmdPkg(L6470_Id, L6470_AppCmdPkg, L6470_GOHOME_ID, 0, 0, 0);
}

/**
  * @brief  Prepare to send @ref L6470_GoMark command.
  * 
  * @param  L6470_Id      The identifier of the L6470 target inside the daisy chain.
  *
  * @note   This function will properly fill the right column of the L6470_AppCmdPkg.
  * @note   The commad will be sent by @ref L6470_PerformPreparedApplicationCommand.
  */
void L6470_PrepareGoMark(uint8_t L6470_Id)
{
  L6470_PrepareAppCmdPkg(L6470_Id, L6470_AppCmdPkg, L6470_GOMARK_ID, 0, 0, 0);
}

/**
  * @brief  Prepare to send @ref L6470_ResetPos command.
  * 
  * @param  L6470_Id      The identifier of the L6470 target inside the daisy chain.
  *
  * @note   This function will properly fill the right column of the L6470_AppCmdPkg.
  * @note   The commad will be sent by @ref L6470_PerformPreparedApplicationCommand.
  */
void L6470_PrepareResetPos(uint8_t L6470_Id)
{
  L6470_PrepareAppCmdPkg(L6470_Id, L6470_AppCmdPkg, L6470_RESETPOS_ID, 0, 0, 0);
}

/**
  * @brief  Prepare to send @ref L6470_ResetDevice command.
  * 
  * @param  L6470_Id      The identifier of the L6470 target inside the daisy chain.
  *
  * @note   This function will properly fill the right column of the L6470_AppCmdPkg.
  * @note   The commad will be sent by @ref L6470_PerformPreparedApplicationCommand.
  */
void L6470_PrepareResetDevice(uint8_t L6470_Id)
{
  L6470_PrepareAppCmdPkg(L6470_Id, L6470_AppCmdPkg, L6470_RESETDEVICE_ID, 0, 0, 0);
}

/**
  * @brief  Prepare to send @ref L6470_SoftStop command.
  * 
  * @param  L6470_Id      The identifier of the L6470 target inside the daisy chain.
  *
  * @note   This function will properly fill the right column of the L6470_AppCmdPkg.
  * @note   The commad will be sent by @ref L6470_PerformPreparedApplicationCommand.
  */
void L6470_PrepareSoftStop(uint8_t L6470_Id)
{
  L6470_PrepareAppCmdPkg(L6470_Id, L6470_AppCmdPkg, L6470_SOFTSTOP_ID, 0, 0, 0);
}

/**
  * @brief  Prepare to send @ref L6470_HardStop command.
  * 
  * @param  L6470_Id      The identifier of the L6470 target inside the daisy chain.
  *
  * @note   This function will properly fill the right column of the L6470_AppCmdPkg.
  * @note   The commad will be sent by @ref L6470_PerformPreparedApplicationCommand.
  */
void L6470_PrepareHardStop(uint8_t L6470_Id)
{
  L6470_PrepareAppCmdPkg(L6470_Id, L6470_AppCmdPkg, L6470_HARDSTOP_ID, 0, 0, 0);
}

/**
  * @brief  Prepare to send @ref L6470_SoftHiZ command.
  * 
  * @param  L6470_Id      The identifier of the L6470 target inside the daisy chain.
  *
  * @note   This function will properly fill the right column of the L6470_AppCmdPkg.
  * @note   The commad will be sent by @ref L6470_PerformPreparedApplicationCommand.
  */
void L6470_PrepareSoftHiZ(uint8_t L6470_Id)
{
  L6470_PrepareAppCmdPkg(L6470_Id, L6470_AppCmdPkg, L6470_SOFTHIZ_ID, 0, 0, 0);
}

/**
  * @brief  Prepare to send @ref L6470_HardHiZ command.
  * 
  * @param  L6470_Id      The identifier of the L6470 target inside the daisy chain.
  *
  * @note   This function will properly fill the right column of the L6470_AppCmdPkg.
  * @note   The commad will be sent by @ref L6470_PerformPreparedApplicationCommand.
  */
void L6470_PrepareHardHiZ(uint8_t L6470_Id)
{
  L6470_PrepareAppCmdPkg(L6470_Id, L6470_AppCmdPkg, L6470_HARDHIZ_ID, 0, 0, 0);
}

/**
  * @brief  Prepare to send @ref L6470_GetStatus command.
  * 
  * @param  L6470_Id      The identifier of the L6470 target inside the daisy chain.
  *
  * @note   This function will properly fill the right column of the L6470_AppCmdPkg.
  * @note   The commad will be sent by @ref L6470_PerformPreparedApplicationCommand.
  */
void L6470_PrepareGetStatus(uint8_t L6470_Id)
{
  L6470_PrepareAppCmdPkg(L6470_Id, L6470_AppCmdPkg, L6470_GETSTATUS_ID, 0, 0, 0);
}

/**
  * @}
  */ /* End of L6470_AppCMDs_ToBePrepared */

/**
  * @brief  Send via SPI the command stored inside the L6470_AppCmdPkg to the
  *         L6470 daisy chain.
  * 
  * @retval (uint8_t*)L6470_DaisyChainSpiRxStruct  The pointer to the structure
  *         containing returned values from each L6470 of the daisy chain for each
  *         sent SPI data.
  */
uint8_t* L6470_PerformPreparedApplicationCommand(void)
{
  L6470_PrepareDaisyChainCommand(L6470_AppCmdPkg, (uint8_t*)L6470_DaisyChainSpiTxStruct);
  L6470_DaisyChainCommand((uint8_t*)L6470_DaisyChainSpiTxStruct, (uint8_t*)L6470_DaisyChainSpiRxStruct);
  
  return (uint8_t*)(L6470_DaisyChainSpiRxStruct);
}

/**
  * @brief  Address the L6470 inside the daisy chain starting from '0'.
  * 
  * @param  position  The position of the L6470 inside the daisy chain starting from '1'. 
  * 
  * @retval id        The column to address inside the L6470_DaisyChainSpiTxStruct.
  *
  * @note   The position '1' is close the uC MISO.
  * @note   If the position is not allowable the returned value is 0xFF.
  */
uint8_t L6470_ID(uint8_t position)
{
   if ((position == 0) || ((position > 0) && (position < L6470DAISYCHAINSIZE)))
  {
    return (position);
  }
  else
  {
    return 0xFF;
  }
}

/**
  * @brief  Send command to the L6470 daisy chain via SPI
  * @param  pL6470_DaisyChainSpiTxStruct  Pointer to the matrix array of bytes to be sent to the daisy chain L6470
  * @param  pL6470_DaisyChainSpiRxStruct  Pointer to the matrix array of bytes to be received from the daisy chain L6470
  */
void L6470_DaisyChainCommand(uint8_t* pL6470_DaisyChainSpiTxStruct, uint8_t* pL6470_DaisyChainSpiRxStruct)
{
  uint8_t spibyte;
  
  L6470_DaisyChain_HalfPrepared = ZERO_F;
  
  /* Send all command bytes via SPI */
  for(spibyte=0; spibyte < L6470MAXSPICMDBYTESIZE; spibyte++)
  {
    /* Enable SPI communication for L6470 */
    L6470_nCS_LOW();
    
    /* Send the command via SPI */
    L6470_SPI_Communication(&HSPI, (pL6470_DaisyChainSpiTxStruct+(spibyte * L6470DAISYCHAINSIZE)), (pL6470_DaisyChainSpiRxStruct+(spibyte * L6470DAISYCHAINSIZE)), L6470DAISYCHAINSIZE, 10);
    
    /* Allow the device to decode the received command */
    L6470_nCS_HIGH();
    
    _DELAY(TDISCS);
  }
}

/**
  * @brief    Extracts the data returned by the L6470 from the matrix that
  *           contains the received SPI data.
  * @param    L6470_Id  The identifier of the L6470 target inside the daisy chain.
  * @param    pL6470_DaisyChainSpiRxStruct  uint8_t-pointer to the matrix that
  *           contains the received data by SPI from the L6470 daisy chain.
  * @param    LengthByte  The number of bytes about the received value.
  */
uint32_t L6470_ExtractReturnedData(uint8_t L6470_Id, uint8_t* pL6470_DaisyChainSpiRxStruct, uint8_t LengthByte)
{
  uint32_t value;
  uint8_t i;
  
  value = 0x000000;
  for (i=1; i<=(L6470MAXSPICMDBYTESIZE-1); i++)
  {
    value |= (*(pL6470_DaisyChainSpiRxStruct+(i*L6470DAISYCHAINSIZE)+L6470_Id))<<((LengthByte-i)*8);
  }
  
  return value;
}

/**
  * @brief  Check the state of a flag inside the L6470 STATUS register.
  *
  * @param  L6470_Id  The identifier of the L6470 target inside the daisy chain.
  * @param  L6470_StatusRegisterFlagId  The identifier of the L6470 STATUS flag to be checked.
  *
  * @retval state The flag state.
  */

uint8_t L6470_CheckStatusRegisterFlag(uint8_t L6470_Id, uint8_t L6470_StatusRegisterFlagId)
{
  uint8_t state = 0;
  
  *((uint16_t*)pL6470_StatusRegister) = L6470_GetParam(L6470_Id, L6470_STATUS_ID);
  
  switch(L6470_StatusRegisterFlagId)
  {
  case HiZ_ID:
    state = pL6470_StatusRegister->HiZ;
    break;
  case BUSY_ID:
    state = pL6470_StatusRegister->BUSY;
    break;
  case SW_F_ID:
    state = pL6470_StatusRegister->SW_F;
    break;
  case SW_EVN_ID:
    state = pL6470_StatusRegister->SW_EVN;
    break;
  case DIR_ID:
    state = pL6470_StatusRegister->DIR;
    break;
  case MOT_STATUS_ID:
    state = pL6470_StatusRegister->MOT_STATUS;
    break;
  case NOTPERF_CMD_ID:
    state = pL6470_StatusRegister->NOTPERF_CMD;
    break;
  case WRONG_CMD_ID:
    state = pL6470_StatusRegister->WRONG_CMD;
    break;
  case UVLO_ID:
    state = pL6470_StatusRegister->UVLO;
    break;
  case TH_WRN_ID:
    state = pL6470_StatusRegister->TH_WRN;
    break;
  case TH_SD_ID:
    state = pL6470_StatusRegister->TH_SD;
    break;
  case OCD_ID:
    state = pL6470_StatusRegister->OCD;
    break;
  case STEP_LOSS_A_ID:
    state = pL6470_StatusRegister->STEP_LOSS_A;
    break;
  case STEP_LOSS_B_ID:
    state = pL6470_StatusRegister->STEP_LOSS_B;
    break;
  case SCK_MOD_ID:
    state = pL6470_StatusRegister->SCK_MOD;
    break;
  }
  
  return state;
}

/**
  * @brief  Return the mnemonic name for the L6470 register.
  * @param  id  The identifier of the L6470 register.
  */
uint8_t *L6470_GetRegisterName(uint8_t id)
{
  if (id < L6470REGIDSIZE)
  {
    return (uint8_t*)L6470_Register[id].Name;
  }
  else
  {
    return "";
  }
}

/**
  * @brief  Configures the L6470 registers.
  * @param  StepperMotorDriverHandle_t* The pointer to the stepper motor driver handle structure.
  * @param  MotorParameterData_t* The pointer to the related parameter data structure.
  */
void L6470_Config(StepperMotorDriverHandle_t *StepperMotorDriverHandle, MotorParameterData_t *MotorParameterData)
{
  /* Prepare the 'Register' field of StepperMotorDriverHandle */
  StepperMotorDriverHandle->Register.ACC = Step_s2_2_Acc(MotorParameterData->acc);
  StepperMotorDriverHandle->Register.DEC = Step_s2_2_Dec(MotorParameterData->dec);
  StepperMotorDriverHandle->Register.MAX_SPEED = Step_s_2_MaxSpeed(MotorParameterData->maxspeed);
  StepperMotorDriverHandle->Register.MIN_SPEED = Step_s_2_MinSpeed(MotorParameterData->minspeed);
  StepperMotorDriverHandle->Register.FS_SPD = Step_s_2_FsSpd(MotorParameterData->fsspd);
  StepperMotorDriverHandle->Register.KVAL_HOLD = (uint8_t)((float)((float)(MotorParameterData->kvalhold * 256) / (MotorParameterData->motorvoltage)));
  StepperMotorDriverHandle->Register.KVAL_RUN = (uint8_t)((float)((float)(MotorParameterData->kvalrun * 256) / (MotorParameterData->motorvoltage)));
  StepperMotorDriverHandle->Register.KVAL_ACC = (uint8_t)((float)((float)(MotorParameterData->kvalacc * 256) / (MotorParameterData->motorvoltage)));
  StepperMotorDriverHandle->Register.KVAL_DEC = (uint8_t)((float)((float)(MotorParameterData->kvaldec * 256) / (MotorParameterData->motorvoltage)));
  StepperMotorDriverHandle->Register.INT_SPEED = Step_s_2_IntSpeed(MotorParameterData->intspeed);
  StepperMotorDriverHandle->Register.ST_SLP = s_Step_2_StSlp(MotorParameterData->stslp);
  StepperMotorDriverHandle->Register.FN_SLP_ACC = s_Step_2_FnSlpAcc(MotorParameterData->fnslpacc);
  StepperMotorDriverHandle->Register.FN_SLP_DEC = s_Step_2_FnSlpDec(MotorParameterData->fnslpdec);
  StepperMotorDriverHandle->Register.K_THERM = MotorParameterData->kterm;
  StepperMotorDriverHandle->Register.OCD_TH = mA_2_OcdTh(MotorParameterData->ocdth);
  StepperMotorDriverHandle->Register.STALL_TH = mA_2_StallTh(MotorParameterData->stallth);
  StepperMotorDriverHandle->Register.STEP_MODE = MotorParameterData->step_sel;
  StepperMotorDriverHandle->Register.ALARM_EN = MotorParameterData->alarmen;
  StepperMotorDriverHandle->Register.CONFIG = MotorParameterData->config;
  
  /* Write the L6470 registers with the prepared data */
  StepperMotorDriverHandle->Command->SetParam(StepperMotorDriverHandle->DaisyChainPosition, L6470_ACC_ID, StepperMotorDriverHandle->Register.ACC);
  StepperMotorDriverHandle->Command->SetParam(StepperMotorDriverHandle->DaisyChainPosition, L6470_DEC_ID, StepperMotorDriverHandle->Register.DEC);
  StepperMotorDriverHandle->Command->SetParam(StepperMotorDriverHandle->DaisyChainPosition, L6470_MAX_SPEED_ID, StepperMotorDriverHandle->Register.MAX_SPEED);
  StepperMotorDriverHandle->Command->SetParam(StepperMotorDriverHandle->DaisyChainPosition, L6470_MIN_SPEED_ID, StepperMotorDriverHandle->Register.MIN_SPEED);
  StepperMotorDriverHandle->Command->SetParam(StepperMotorDriverHandle->DaisyChainPosition, L6470_FS_SPD_ID, StepperMotorDriverHandle->Register.FS_SPD);
  StepperMotorDriverHandle->Command->SetParam(StepperMotorDriverHandle->DaisyChainPosition, L6470_KVAL_HOLD_ID, StepperMotorDriverHandle->Register.KVAL_HOLD);
  StepperMotorDriverHandle->Command->SetParam(StepperMotorDriverHandle->DaisyChainPosition, L6470_KVAL_RUN_ID, StepperMotorDriverHandle->Register.KVAL_RUN);
  StepperMotorDriverHandle->Command->SetParam(StepperMotorDriverHandle->DaisyChainPosition, L6470_KVAL_ACC_ID, StepperMotorDriverHandle->Register.KVAL_ACC);
  StepperMotorDriverHandle->Command->SetParam(StepperMotorDriverHandle->DaisyChainPosition, L6470_KVAL_DEC_ID, StepperMotorDriverHandle->Register.KVAL_DEC);
  StepperMotorDriverHandle->Command->SetParam(StepperMotorDriverHandle->DaisyChainPosition, L6470_INT_SPEED_ID, StepperMotorDriverHandle->Register.INT_SPEED);
  StepperMotorDriverHandle->Command->SetParam(StepperMotorDriverHandle->DaisyChainPosition, L6470_ST_SLP_ID, StepperMotorDriverHandle->Register.ST_SLP);
  StepperMotorDriverHandle->Command->SetParam(StepperMotorDriverHandle->DaisyChainPosition, L6470_FN_SLP_ACC_ID, StepperMotorDriverHandle->Register.FN_SLP_ACC);
  StepperMotorDriverHandle->Command->SetParam(StepperMotorDriverHandle->DaisyChainPosition, L6470_FN_SLP_DEC_ID, StepperMotorDriverHandle->Register.FN_SLP_DEC);
  StepperMotorDriverHandle->Command->SetParam(StepperMotorDriverHandle->DaisyChainPosition, L6470_K_THERM_ID, StepperMotorDriverHandle->Register.K_THERM);
  StepperMotorDriverHandle->Command->SetParam(StepperMotorDriverHandle->DaisyChainPosition, L6470_OCD_TH_ID, StepperMotorDriverHandle->Register.OCD_TH);
  StepperMotorDriverHandle->Command->SetParam(StepperMotorDriverHandle->DaisyChainPosition, L6470_STALL_TH_ID, StepperMotorDriverHandle->Register.STALL_TH);
  StepperMotorDriverHandle->Command->SetParam(StepperMotorDriverHandle->DaisyChainPosition, L6470_STEP_MODE_ID, StepperMotorDriverHandle->Register.STEP_MODE);
  StepperMotorDriverHandle->Command->SetParam(StepperMotorDriverHandle->DaisyChainPosition, L6470_ALARM_EN_ID, StepperMotorDriverHandle->Register.ALARM_EN);
  StepperMotorDriverHandle->Command->SetParam(StepperMotorDriverHandle->DaisyChainPosition, L6470_CONFIG_ID, StepperMotorDriverHandle->Register.CONFIG);
}

/**
  * @}
  */ /* End of L6470_Exported_Functions */

/**
  * @}
  */ /* End of L6470 */

/**
  * @}
  */ /* End of Components */

/**
  * @}
  */ /* End of BSP */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
