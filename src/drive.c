/*
 * drive.c
 *
 *  Created on: 04 окт. 2016 г.
 *      Author: Gennady Tanchin <g.tanchin@yandex.ru>
 */

#include "drive.h"
#include "can.h"
#include "my_time.h"

volatile tValve valve;
int32_t startHollCount = 0;
uint8_t startDeg = 0;
uint8_t deltaCount = 0;

extern RCC_ClocksTypeDef RCC_Clocks;

static int drivePwmInit( void );

int valveInit( void ){
	int ret = 0;

#warning " !!! Указать, какому контуру принадлежит контроллер (Горячий/Холодный)"
  valve.coldHot = COLD;
  valve.err = ERR_OK;
  valve.dir = DIR_STOP;
  valve.state = STATE_CALIBRATE;
  valve.adjDeg = 0;
  valve.curDeg = 0;
	return ret;
}

int driveInit( void ){
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

	// TIM PWM Init
	drivePwmInit();
	driveStop();

	// Enable the TIM Main Output
  DRIVE_TIM->BDTR |= TIM_BDTR_MOE;
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

static int drivePwmInit( void ){
	int ret = 0;
	TIM_TimeBaseInitTypeDef DRIVE_TimeBaseStructure;
	TIM_OCInitTypeDef DRIVE_OCInitStructure;
	uint32_t clkDiv;

	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	TIM_DeInit(DRIVE_TIM);

  // Enable the TIM Main Output
  DRIVE_TIM->BDTR &= ~TIM_BDTR_MOE;

  //Находим частоту тактирования таймера
	clkDiv = (RCC_Clocks.PCLK2_Frequency * (((RCC->CFGR & RCC_CFGR_PPRE1) & 0x4)?2:1));
	// Частота ШИМ = 200000 Гц
	clkDiv /= 200000;
	DRIVE_TimeBaseStructure.TIM_Prescaler = clkDiv - 1;
	DRIVE_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	DRIVE_TimeBaseStructure.TIM_Period = FULL_SPEED - 1;
	DRIVE_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	DRIVE_TimeBaseStructure.TIM_RepetitionCounter = 0x00;
	TIM_TimeBaseInit(DRIVE_TIM, &DRIVE_TimeBaseStructure);

	DRIVE_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	DRIVE_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	DRIVE_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	DRIVE_OCInitStructure.TIM_Pulse = 0x0000;
	DRIVE_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	DRIVE_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	DRIVE_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	DRIVE_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
	TIM_OC1Init(DRIVE_TIM, &DRIVE_OCInitStructure);
	TIM_OC2Init(DRIVE_TIM, &DRIVE_OCInitStructure);

	// Enable the TIM Counter
	DRIVE_TIM->CR1 |= TIM_CR1_CEN;

	return ret;
}

TIM_BDTRInitTypeDef DRIVE_TIM_BDTRInitStructure;

int hollInit( void ){
	int ret = 0;
	EXTI_InitTypeDef holl_EXTI_InitStructure;
	GPIO_InitTypeDef holl_InitStructure;

	HOLL_PORT_CLK_ENABLE;

	holl_InitStructure.GPIO_Pin = HOLL_SIN_PIN | HOLL_COS_PIN;
	holl_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	holl_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(HOLL_PORT, &holl_InitStructure);

	GPIO_EXTILineConfig(HOLL_PORTSOURCE, HOLL_PINSOURCE);
	holl_EXTI_InitStructure.EXTI_Line = HOLL_EXTI_LINE;
	holl_EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	holl_EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	holl_EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&holl_EXTI_InitStructure);

	NVIC_EnableIRQ(HOLL_IRQCHANNEL);
	NVIC_SetPriority(HOLL_IRQCHANNEL, 3);

	return ret;
}

int switchInit( void ){
	int ret = 0;
	EXTI_InitTypeDef switch_EXTI_InitStructure;
	GPIO_InitTypeDef switch_InitStructure;

	SWITCH_OPEN_PORT_CLK_ENABLE;
	SWITCH_CLOSE_PORT_CLK_ENABLE;

	switch_InitStructure.GPIO_Pin = SWITCH_OPEN_PIN | SWITCH_CLOSE_PIN;
	switch_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	switch_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(SWITCH_OPEN_PORT, &switch_InitStructure);

	GPIO_EXTILineConfig(SWITCH_OPEN_PORTSOURCE, SWITCH_OPEN_PINSOURCE);

	switch_EXTI_InitStructure.EXTI_Line = SWITCH_OPEN_EXTI_LINE;
	switch_EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	switch_EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	switch_EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&switch_EXTI_InitStructure);

	GPIO_EXTILineConfig(SWITCH_OPEN_PORTSOURCE, SWITCH_CLOSE_PINSOURCE);
	switch_EXTI_InitStructure.EXTI_Line = SWITCH_CLOSE_EXTI_LINE;
	switch_EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	switch_EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	switch_EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&switch_EXTI_InitStructure);

	NVIC_EnableIRQ(SWITCH_IRQCHANNEL);
	NVIC_SetPriority(SWITCH_IRQCHANNEL, 2);

	// Проверка концевиков
	if( SWITCH_OPEN_PORT->IDR & SWITCH_OPEN_PIN ){
		if( SWITCH_CLOSE_PORT->IDR & SWITCH_CLOSE_PIN ){
			valve.err = ERR_SWITCH;
		}
		else {
			valve.curDeg = 90;
		}
	}
	if( SWITCH_CLOSE_PORT->IDR & SWITCH_CLOSE_PIN ){
		if( SWITCH_OPEN_PORT->IDR & SWITCH_OPEN_PIN ){
			valve.err = ERR_SWITCH;
		}
		else {
			valve.curDeg = 0;
		}
	}
	return ret;
}

void driveStart( eDirect dir, eSpeed speed ) {
	if ( ( dir == DIR_FOREWARD) && (valve.sw != SW_OPEN) ) {
		if (valve.dir == DIR_BACKWARD)
		{
			driveStop();
		}
		DRIVE_TIM_CH1_PULSE(0);
		DRIVE_TIM_CH1N_PULSE(FULL_SPEED-speed);
		valve.dir = DIR_FOREWARD;
	}
	else if ((dir == DIR_BACKWARD) && (valve.sw != SW_CLOSE) ) {
		if (valve.dir == DIR_FOREWARD)
		{
			driveStop();
		}
		DRIVE_TIM_CH2_PULSE(1);
		DRIVE_TIM_CH2N_PULSE(speed);
		valve.dir = DIR_BACKWARD;
	}
}

void driveStop(void) {
	DRIVE_TIM_CH1_PULSE(1);
	DRIVE_TIM_CH2_PULSE(0);
	DRIVE_TIM_CH1N_PULSE(FULL_SPEED);
	DRIVE_TIM_CH2N_PULSE(0);
	myDelay(50);
	valve.dir = DIR_STOP;
}

int valveCalibrate( void ){
	int ret = 0;

	// Закрываем задвижку
	driveStart( DIR_BACKWARD, FULL_SPEED );
	while( valve.sw != SW_CLOSE )
	{}
	// Задвижка закрыта
	startHollCount = -valve.hollCount;
	valve.hollCount = 0;
	valve.curDeg = 0;
	// Открываем задвижку
	driveStart( DIR_FOREWARD, FULL_SPEED );
	while( valve.sw != SW_OPEN )
	{}
	// Задвижка открыта
	deltaCount = valve.hollCount / 90;
	valve.curDeg = 90;
	valve.adjDeg = startHollCount / deltaCount;

	return ret;
}

int driveProcess( void ){
	int ret = 0;

	switch( valve.state ){
		case STATE_NEED_STOP:
			driveStop();
			valve.state = STATE_WORK;
			break;
		case STATE_CALIBRATE:
			valveCalibrate();
			valve.state = STATE_WORK;
			break;
		case STATE_WORK:
		case STATE_CORRECT:
			valve.curDeg = valve.hollCount / deltaCount;
			if( valve.curDeg < valve.adjDeg ){
				if( valve.dir == DIR_BACKWARD ){
					driveStop();
				}
				else if( valve.dir == DIR_STOP ){
					driveStart( DIR_FOREWARD, FULL_SPEED );
				}
			}
			else if( valve.curDeg > valve.adjDeg ){
				if( valve.dir == DIR_FOREWARD ){
					driveStop();
				}
				else if( valve.dir == DIR_STOP ){
					driveStart( DIR_BACKWARD, FULL_SPEED );
				}
			}
			else {
				if( valve.dir != DIR_STOP) {
					driveStop();
				}
				else {
					valve.state = STATE_CORRECT;
					canSendMsg( VALVE_DEG, valve.curDeg );
				}
			}
			break;
		default:
			// Сюда не должны попадать
			break;
	}


	return ret;
}
