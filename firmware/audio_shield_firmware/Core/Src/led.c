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
 * led.c - led/Buzzer driver
 *
 * This code mainly interfacing the PWM peripheral lib of ST.
 *
 *
 * Modified by Adrien Hoffet to embed led buzzer into the audiodeck V3
 *
 */

#include <stdbool.h>

#include "led.h"

// HW defines
//#define led_TIM_PERIF       			RCC_APB1Periph_TIM5
#define LED_TIM             			TIM1
#define LED_hTIM             			htim1
#define LED_TIM_DBG         			DBGMCU_TIM1_STOP
#define LED_TIM_SETCOMPARE_1(VAL) 		LED_TIM->CCR3  = VAL;
#define LED_TIM_SETCOMPARE_2(VAL) 		LED_TIM->CCR1  = VAL;
#define LED_TIM_SETCOMPARE_3(VAL) 		LED_TIM->CCR2  = VAL;
#define LED_TIM_SETCOMPARE_4(VAL) 		LED_TIM->CCR4  = VAL;
#define LED_TIM_SETARR(VAL) 			LED_TIM->ARR  = VAL;
#define LED_TIM_GETCOMPARE 			LED_TIM->CCR3;
#define LED_TIM_SETPSC(VAL)			LED_TIM->PSC = VAL;

/*
 Removed since init is done by cubeMX
 #define LED_GPIO_POS_PERIF         	RCC_AHB1Periph_GPIOA
 #define LED_GPIO_POS_PORT          	GPIOA
 #define LED_GPIO_POS_PIN           	GPIO_Pin_2 // TIM5_CH3
 #define LED_GPIO_AF_POS_PIN        	GPIO_PinSource2
 #define LED_GPIO_AF_POS            	GPIO_AF_TIM5

 #define LED_GPIO_NEG_PERIF         	RCC_AHB1Periph_GPIOA
 #define LED_GPIO_NEG_PORT          	GPIOA
 #define LED_GPIO_NEG_PIN           	GPIO_Pin_3 // TIM5_CH4
 #define LED_GPIO_AF_NEG_PIN        	GPIO_PinSource3
 #define LED_GPIO_AF_NEG            	GPIO_AF_TIM5
 */

#define LED_PWM_BITS      			(8)
#define LED_PWM_PERIOD    			((1<<LED_PWM_BITS) - 1)
#define LED_PWM_PRESCALE  			(0)

/* This should be calculated.. */
#define LED_BASE_FREQ (84000000)

static bool isInit = false;

/* Public functions */

void ledInit() {
	if (isInit)
		return;

	// PWM default configuration
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0x0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		//return -1;
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		//return -1;
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		//return -1;
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		//return -1;
	}

	// Restart the PWM since it is automatically disabled when modified
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	isInit = true;
}

bool ledTest(void) {
	return isInit;
}

void ledSetMaxCount(uint16_t max) {
	LED_TIM_SETARR(max);
}

void ledSetRatio(uint16_t ratio, uint8_t led) {
	switch (led) {
	case 1:
		LED_TIM_SETCOMPARE_1(ratio);
		break;
	case 2:
		LED_TIM_SETCOMPARE_2(ratio);
		break;
	case 3:
		LED_TIM_SETCOMPARE_3(ratio);
		break;
	case 4:
		LED_TIM_SETCOMPARE_4(ratio);
		break;
	}

}

void ledSetPSC(uint16_t psc) {
	LED_TIM->PSC = psc;
}
