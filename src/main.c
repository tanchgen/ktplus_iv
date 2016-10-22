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
#include "can.h"
#include "drive.h"


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

  while (1) {
 		timersProcess();
 		canProcess();
 		driveProcess();

/*
  	 while( (buttonCount & 0x1) == 0 )
  	 {}
  	 driveStart( DIR_BACKWARD, FULL_SPEED );
  	 myDelay(1000);
  	 driveStart( DIR_FOREWARD, FULL_SPEED );
  	 myDelay(1000);
  	 driveStart( DIR_BACKWARD, HALF_SPEED );
  	 myDelay(1000);
  	 driveStart( DIR_FOREWARD, HALF_SPEED );
  	 myDelay(1000);
  	 driveStart( DIR_BACKWARD, QUART_SPEED );
  	 myDelay(1000);
  	 driveStart( DIR_FOREWARD, QUART_SPEED );
  	 myDelay(1000);
  	 driveStop();
  	 driveStart( DIR_FOREWARD, FULL_SPEED );
  	 myDelay(3000);
  	 driveStop();
  	 driveStart( DIR_FOREWARD, HALF_SPEED );
  	 myDelay(3000);
  	 driveStop();
  	 driveStart( DIR_FOREWARD, QUART_SPEED );
  	 myDelay(3000);

   	 driveStop();
   	 */
  }
}

/*
void driveBspInit( void ){
//	CH1_Out 	PE9
// 	CH1N_Out 	PE8
//
// 	CH2_Out		PE11
// 	CH2N_Out	PE10
//

	RCC->APB1ENR |= RCC_AHB1ENR_GPIOEEN;

	// CH1_Out config
	GPIOE->MODER &= ~(3 << (9 * 2));
	GPIOE->MODER |= 2 << (9 * 2);					// AF_OUT
	GPIOE->OTYPER &= ~(GPIO_Pin_9);
	GPIOE->OSPEEDR &= ~(3 << (9 * 2));		// PullUp-PullDown
	GPIOE->OSPEEDR |= 1 << (9 * 2);				// Medium speed
	GPIOE->PUPDR &= ~(3 << (9 * 2));
	GPIOE->PUPDR |= 1 << (9 * 2);					// PullUp
	GPIOE->AFR[1] |= 1<< ((9-8) * 4);					// AF 1

	// CH1N_Out config
	GPIOE->MODER &= ~(3 << (8 * 2));
	GPIOE->MODER |= 2 << (8 * 2);					// AF_OUT
	GPIOE->OTYPER &= ~(GPIO_Pin_8);
	GPIOE->OSPEEDR &= ~(3 << (8 * 2));		// PullUp-PullDown
	GPIOE->OSPEEDR |= 1 << (8 * 2);				// Medium speed
	GPIOE->PUPDR &= ~(3 << (8 * 2));
	GPIOE->PUPDR |= 1 << (8 * 2);					// PullUp
	GPIOE->AFR[1] |= 1<< ((8-8) * 4);					// AF 1

	// CH2_Out config
	GPIOE->MODER &= ~(3 << (11 * 2));
	GPIOE->MODER |= 2 << (11 * 2);					// AF_OUT
	GPIOE->OTYPER &= ~(GPIO_Pin_11);
	GPIOE->OSPEEDR &= ~(3 << (11 * 2));		// PullUp-PullDown
	GPIOE->OSPEEDR |= 1 << (11 * 2);				// Medium speed
	GPIOE->PUPDR &= ~(3 << (11 * 2));
	GPIOE->PUPDR |= 1 << (11 * 2);					// PullUp
	GPIOE->AFR[1] |= 1<< ((11-8) * 4);					// AF 1

	// CH2N_Out config
	GPIOE->MODER &= ~(3 << (10 * 2));
	GPIOE->MODER |= 2 << (10 * 2);					// AF_OUT
	GPIOE->OTYPER &= ~(GPIO_Pin_10);
	GPIOE->OSPEEDR &= ~(3 << (10 * 2));		// PullUp-PullDown
	GPIOE->OSPEEDR |= 1 << (10 * 2);				// Medium speed
	GPIOE->PUPDR &= ~(3 << (10 * 2));
	GPIOE->PUPDR |= 1 << (10 * 2);					// PullUp
	GPIOE->AFR[1] |= 1<< ((10-8) * 4);					// AF 1
}
*/

void driveTimInit( void ){

	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	RCC->APB2RSTR |= RCC_APB2RSTR_TIM1RST;
	RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM1RST;

	TIM1->CR1 &= ~TIM_CR1_CEN;
	TIM1->CR1 &= TIM_CR1_DIR;
	TIM1->PSC = (RCC_Clocks.PCLK2_Frequency * 2)/1000000 - 1;
	TIM1->ARR = 1000;

	TIM1->CCMR1 &= ~TIM_CCMR1_OC1PE;
	TIM1->CCMR1 &= ~TIM_CCMR1_OC2PE;

	TIM1->CCER &= ~TIM_CCER_CC1P;
	TIM1->CCER &= ~TIM_CCER_CC1NP;

	TIM1->CCER &= ~TIM_CCER_CC2P;
	TIM1->CCER &= ~TIM_CCER_CC2NP;

	driveStop();
	TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1NE;
	TIM1->CCER |= TIM_CCER_CC2E | TIM_CCER_CC2NE;

	// Запускаем таймер
	TIM1->CR1 |= TIM_CR1_CEN;
}


// ----------------------------------------------------------------------------
