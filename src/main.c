//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f10x.h"
#include "my_time.h"
#include "drive.h"
#include "main.h"

uint32_t VlvDevId;			// Иденитификатор контроллера задвижки

RCC_ClocksTypeDef RCC_Clocks;


// ----- main() ---------------------------------------------------------------

int main(int argc, char* argv[]) {
	(void)argc;
	(void)argv;
	myTick = 0;
  uxTime = 1472731200;			// Unix Time = 01.09.2016г., четверг

  RCC_GetClocksFreq(&RCC_Clocks);
  // Use SysTick as reference for the delay loops.
  SysTick_Config (SystemCoreClock / TIMER_FREQUENCY_HZ);

  //Инициализация концевиков
  switchInit();
  // Инициализация датчика Холла
  hollInit();
  // Инициализация двигателя
  driveInit();
  // Инициализация CAN
  canInit();
  // Инциализация таймера дребезга
  debounceInit();
  // Общая инициализация контроллера
  valveInit();


  // Infinite loop
  while (1)
    {
  		timersProcess();
  		canProcess();
  		driveProcess();
    }
}

// ----------------------------------------------------------------------------
