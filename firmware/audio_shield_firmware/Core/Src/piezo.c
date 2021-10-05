/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * piezo.c - Piezo/Buzzer driver
 *
 * This code mainly interfacing the PWM peripheral lib of ST.
 *
 *
 * Modified by Adrien Hoffet to embed piezo buzzer into the audiodeck V3
 *
 */

#include <stdbool.h>

#include "piezo.h"


// HW defines
//#define PIEZO_TIM_PERIF       			RCC_APB1Periph_TIM5
#define PIEZO_TIM             			TIM3
#define PIEZO_hTIM             			htim3
#define PIEZO_CHANNEL					TIM_CHANNEL_1
#define PIEZO_TIM_DBG         			DBGMCU_TIM5_STOP
#define PIEZO_TIM_SETCOMPARE(VAL) 		PIEZO_TIM->CCR1  = VAL;
#define PIEZO_TIM_SETARR(VAL) 			PIEZO_TIM->ARR  = VAL;
#define PIEZO_TIM_GETCOMPARE 			PIEZO_TIM->CCR1;
#define PIEZO_TIM_SETPSC(VAL)			PIEZO_TIM->PSC = VAL;



/*
 Removed since init is done by cubeMX
#define PIEZO_GPIO_POS_PERIF         	RCC_AHB1Periph_GPIOA
#define PIEZO_GPIO_POS_PORT          	GPIOA
#define PIEZO_GPIO_POS_PIN           	GPIO_Pin_2 // TIM5_CH3
#define PIEZO_GPIO_AF_POS_PIN        	GPIO_PinSource2
#define PIEZO_GPIO_AF_POS            	GPIO_AF_TIM5

#define PIEZO_GPIO_NEG_PERIF         	RCC_AHB1Periph_GPIOA
#define PIEZO_GPIO_NEG_PORT          	GPIOA
#define PIEZO_GPIO_NEG_PIN           	GPIO_Pin_3 // TIM5_CH4
#define PIEZO_GPIO_AF_NEG_PIN        	GPIO_PinSource3
#define PIEZO_GPIO_AF_NEG            	GPIO_AF_TIM5
*/

#define PIEZO_PWM_BITS      			(8)
#define PIEZO_PWM_PERIOD    			((1<<PIEZO_PWM_BITS) - 1)
#define PIEZO_PWM_PRESCALE  			(0)

/* This should be calculated.. */
#define PIEZO_BASE_FREQ (84000000)

static bool isInit = false;

/* Public functions */

void piezoInit()
{
	if (isInit)
		return;

	// PWM default configuration
	TIM_OC_InitTypeDef sConfigOC = {0};

	sConfigOC.OCMode 		= TIM_OCMODE_PWM1;
	sConfigOC.Pulse 		= 0xFFFF;
	sConfigOC.OCPolarity 	= TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode 	= TIM_OCFAST_DISABLE;

	if (HAL_TIM_PWM_ConfigChannel(&PIEZO_hTIM, &sConfigOC, PIEZO_CHANNEL) != HAL_OK)
	{
		//return -1;
	}

	// Restart the PWM since it is automatically disabled when modified
	piezoStart();

	isInit = true;
}

void piezoStart()
{
	HAL_TIM_PWM_Start(&PIEZO_hTIM, PIEZO_CHANNEL);
}

bool piezoTest(void)
{
  return isInit;
}

void piezoSetMaxCount(uint16_t max)
{
	PIEZO_TIM_SETARR(max);
	isInit = false;
}

void piezoSetRatio(uint16_t ratio)
{
	PIEZO_TIM_SETCOMPARE(ratio);
}

void piezoSetPSC(uint16_t psc)
{
	PIEZO_TIM_SETPSC(psc);
}
