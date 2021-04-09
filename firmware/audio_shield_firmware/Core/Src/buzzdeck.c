/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2015 BitCraze AB
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
 * buzzdeck.c - Deck driver for the buzzer deck
 */

#include <stdint.h>
#include <stdlib.h>

//#include "deck.h"
//#include "param.h"

#include "buzzer.h"
#include "piezo.h"

static bool isInit;

static void buzzDeckOn(uint32_t freq)
{
  piezoSetRatio(128);
  piezoSetFreq(freq);
}

static void buzzDeckOff()
{
  piezoSetRatio(0);
}


static void buzzDeckInit()
{
  if (isInit) {
    return;
  }

  piezoInit();
  //buzzerSetControl(&buzzDeckCtrl);

  isInit = true;
}


