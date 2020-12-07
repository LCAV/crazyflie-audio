/**
  ******************************************************************************
  * @file    params.c
  * @author  IPD SYSTEM LAB & TECH MKTG
  * @version V0.0.1
  * @date    04-June-2015
  * @brief   This file contains the parameters for the used stepper motors.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
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
#include "params.h"
#include "xnucleoihm02a1.h"

/**
  * @addtogroup   MotionControl
  * @{
  */

/**
  * @addtogroup MicrosteppingMotor_Example
  * @{
  */

/**
  * @addtogroup Attached_Motor_Parameters
  * @{
  */

/**
  * @addtogroup Parameters_Exported_Constants
  * @{
  */

/**
  * @brief  This array contains the initial parameters for the motors that are
  *         linked with each L6470 mounted on each stacked X-NUCLEO-IHM02A1.
  */
const MotorParameterData_t MotorParameterInitData[EXPBRD_MOUNTED_NR_MAX][L6470DAISYCHAINSIZE] = {
  {
    // rotational motor
	{12.0, 400, 10, 12, 200.0, 1300.0, 1300.0, 600.0, 0.0, 602.7, 3.06, 3.06,\
	  3.06, 3.06, 61.52, 392.1569e-6, 643.1372e-6, 643.1372e-6, 0,\
		3.06*1000*1.10, 3.06*1000*1.00, MICROSTEP_1_128, 0xFF, 0x2E88},
	// linear motor
	{12.0, 200, 4.6, 2.5, 300, 200.0, 200.0, 300.0, 0.0, 602.7, 3.06, 11,\
		7, 7, 50, 392.1569e-6, 643.1372e-6, 643.1372e-6, 0,\
		6000, 4000, MICROSTEP_1_16, 0xFF, 0x2E88},
  },
  {
    {9.0, 400, 1.7, 3.06, 300.0, 500.0, 500.0, 400.0, 0.0, 602.7, 3.06, 3.06,\
      3.06, 3.06, 61.52, 392.1569e-6, 643.1372e-6, 643.1372e-6, 0,\
        3.06*1000*1.10, 3.06*1000*1.00, MICROSTEP_1_128, 0xFF, 0x2E88},
    {9.0, 400, 1.7, 3.06, 300.0, 500.0, 500.0, 400.0, 0.0, 602.7, 3.06, 3.06,\
      3.06, 3.06, 61.52, 392.1569e-6, 643.1372e-6, 643.1372e-6, 0,\
        3.06*1000*1.10, 3.06*1000*1.00, MICROSTEP_1_128, 0xFF, 0x2E88},
  },
  {
    {9.0, 400, 1.7, 3.06, 300.0, 500.0, 500.0, 400.0, 0.0, 602.7, 3.06, 3.06,\
      3.06, 3.06, 61.52, 392.1569e-6, 643.1372e-6, 643.1372e-6, 0,\
        3.06*1000*1.10, 3.06*1000*1.00, MICROSTEP_1_128, 0xFF, 0x2E88},
    {9.0, 400, 1.7, 3.06, 300.0, 500.0, 500.0, 400.0, 0.0, 602.7, 3.06, 3.06,\
      3.06, 3.06, 61.52, 392.1569e-6, 643.1372e-6, 643.1372e-6, 0,\
        3.06*1000*1.10, 3.06*1000*1.00, MICROSTEP_1_128, 0xFF, 0x2E88},
  },
  {
    {9.0, 400, 1.7, 3.06, 300.0, 500.0, 500.0, 400.0, 0.0, 602.7, 3.06, 3.06,\
      3.06, 3.06, 61.52, 392.1569e-6, 643.1372e-6, 643.1372e-6, 0,\
        3.06*1000*1.10, 3.06*1000*1.00, MICROSTEP_1_128, 0xFF, 0x2E88},
    {9.0, 400, 1.7, 3.06, 300.0, 500.0, 500.0, 400.0, 0.0, 602.7, 3.06, 3.06,\
      3.06, 3.06, 61.52, 392.1569e-6, 643.1372e-6, 643.1372e-6, 0,\
        3.06*1000*1.10, 3.06*1000*1.00, MICROSTEP_1_128, 0xFF, 0x2E88},
  },
};
  
/**
  * @}
  */    /* Parameters_Exported_Constants */

/**
  * @addtogroup Parameters_Exported_Functions
  * @{
  */

/**
  * @brief  Return the initial motor parameters.
  * @param  index   The index inside the array of parameters to point the right data.
  * @retval MotorParameterData_t*   The pointer to the data structure of parameters.
  */
MotorParameterData_t *GetMotorParameterInitData(void)
{
  return (MotorParameterData_t*)(MotorParameterInitData);
}

/**
  * @}
  */    /* Parameters_Exported_Functions */
  
/**
  * @}
  */ /* End of Attached_Motor_Parameters */

/**
  * @}
  */ /* End of MicrosteppingMotor_Example */

/**
  * @}
  */ /* End of MotionControl */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
