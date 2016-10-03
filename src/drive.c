/*
 * drive.c
 *
 *  Created on: 04 окт. 2016 г.
 *      Author: Gennady Tanchin <g.tanchin@yandex.ru>
 */

#include "drive.h"

tValve valve;

static int driveMspInit( void );
static int drivePwmInit( void );

int driveInit( void ){
	int ret = 0;

	driveMspInit();
	drivePwmInit();
	return ret;
}

int hollInit( void ){
	int ret = 0;

	return ret;
}

int testPinInit( void ){
	GPIO_InitTypeDef test_InitStructure;

	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

	test_InitStructure.GPIO_Pin = GPIO_Pin_8;
	test_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	test_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOB, &test_InitStructure);

	return 0;
}

static int driveMspInit( void ){
	int ret = 0;

	GPIO_InitTypeDef drive_InitStructure;

	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

	drive_InitStructure.GPIO_Pin = DRIVE_CH1_PIN | DRIVE_CH2_PIN;
	drive_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	drive_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(DRIVE_PORT, &drive_InitStructure);
	drive_InitStructure.GPIO_Pin = DRIVE_CH1N_PIN | DRIVE_CH2N_PIN;
	drive_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	drive_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(DRIVE_PORT, &drive_InitStructure);
	drive_InitStructure.GPIO_Pin = HOLL_SIN_PIN | HOLL_COS_PIN;
	drive_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	drive_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(DRIVE_PORT, &drive_InitStructure);
	drive_InitStructure.GPIO_Pin = DRIVE_OPEN_PIN | DRIVE_CLOSE_PIN;
	drive_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	drive_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(DRIVE_PORT, &drive_InitStructure);

	return ret;
}

static int drivePwmInit( void ){
	int ret = 0;
	TIM_TimeBaseInitTypeDef DRIVE_TimeBaseStructure;
	TIM_OCInitTypeDef DRIVE_OCInitStructure;

	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	TIM_DeInit(DRIVE_TIM);

	DRIVE_TimeBaseStructure.TIM_Prescaler = 0x0168;
	DRIVE_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	DRIVE_TimeBaseStructure.TIM_Period = 0x03E8;
	DRIVE_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	DRIVE_TimeBaseStructure.TIM_RepetitionCounter = 0x00;
	TIM_TimeBaseInit(DRIVE_TIM, &DRIVE_TimeBaseStructure);
	DRIVE_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	DRIVE_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	DRIVE_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	DRIVE_OCInitStructure.TIM_Pulse = 0x0000;
	DRIVE_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	DRIVE_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	DRIVE_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	DRIVE_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
	TIM_OC1Init(DRIVE_TIM, &DRIVE_OCInitStructure);
	DRIVE_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	DRIVE_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	DRIVE_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	DRIVE_OCInitStructure.TIM_Pulse = 0x0000;
	DRIVE_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	DRIVE_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	DRIVE_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	DRIVE_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
	TIM_OC2Init(DRIVE_TIM, &DRIVE_OCInitStructure);

	// Enable the TIM Counter
	DRIVE_TIM->CR1 |= TIM_CR1_CEN;
  // Enable the TIM Main Output
  DRIVE_TIM->BDTR |= TIM_BDTR_MOE;

	return ret;
}

TIM_BDTRInitTypeDef DRIVE_TIM_BDTRInitStructure;
EXTI_InitTypeDef DRIVE_EXTI_InitStructure;
NVIC_InitTypeDef DRIVE_NVIC_InitStructure;

int hollInit( void ){
	int ret = 0;
	EXTI_InitTypeDef DRIVE_EXTI_InitStructure;

	GPIO_EXTILineConfig(HOLL_PORT, DRIVE_SIN_COS_PINSOURCE);
	DRIVE_EXTI_InitStructure.EXTI_Line = DRIVE_SIN_COS_EXTI_LINE;
	DRIVE_EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	DRIVE_EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	DRIVE_EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&DRIVE_EXTI_InitStructure);
	GPIO_EXTILineConfig(DRIVE_PORTSOURCE, DRIVE_OPEN_PINSOURCE);
	DRIVE_EXTI_InitStructure.EXTI_Line = DRIVE_OPEN_EXTI_LINE;
	DRIVE_EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	DRIVE_EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	DRIVE_EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&DRIVE_EXTI_InitStructure);
	GPIO_EXTILineConfig(DRIVE_PORTSOURCE, DRIVE_CLOSE_PINSOURCE);
	DRIVE_EXTI_InitStructure.EXTI_Line = DRIVE_CLOSE_EXTI_LINE;
	DRIVE_EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	DRIVE_EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	DRIVE_EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&DRIVE_EXTI_InitStructure);
	DRIVE_NVIC_InitStructure.NVIC_IRQChannel = DRIVE_SIN_COS_NVIC_IRQCHANNEL;
	DRIVE_NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	DRIVE_NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	DRIVE_NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&DRIVE_NVIC_InitStructure);
	DRIVE_NVIC_InitStructure.NVIC_IRQChannel = DRIVE_OPEN_CLOSE_NVIC_IRQCHANNEL;
	DRIVE_NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	DRIVE_NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	DRIVE_NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&DRIVE_NVIC_InitStructure);


	return ret;
}
