/*
 * drive.h
 *
 *  Created on: 04 окт. 2016 г.
 *      Author: Gennady Tanchin <g.tanchin@yandex.ru>
 */

#ifndef DRIVE_H_
#define DRIVE_H_

#include "stm32f10x.h"

typedef enum {
	STOP,				// Стоит
	FOREWARD,		// Открывается
	BACKWARD,		// Закрывается
	CORRECT			// Стоит в правильном положении
} eState;

typedef struct {
	uint8_t coldHot;
	uint8_t degCur;
	uint8_t degAdj;
	eState state;
	uint8_t err;
} tValve;

extern tValve valve;

#define DRIVE_RCC_GPIO 										RCC_APB2Periph_GPIOB
#define DRIVE_CH1_PIN 										GPIO_Pin_15
#define DRIVE_CH1N_PIN 										GPIO_Pin_13
#define DRIVE_CH2_PIN 										GPIO_Pin_12
#define DRIVE_CH2N_PIN 										GPIO_Pin_14
#define DRIVE_OPEN_PIN 										GPIO_Pin_7
#define DRIVE_CLOSE_PIN 									GPIO_Pin_6
#define DRIVE_PORT 												GPIOB
#define DRIVE_RCC_TIM 										RCC_APB2Periph_TIM1
#define DRIVE_TIM 												TIM1
#define DRIVE_TIM_CH1_PULSE(X) 						X ? GPIO_WriteBit(DRIVE_PORT, DRIVE_CH1_PIN, Bit_SET) : GPIO_WriteBit(DRIVE_PORT, DRIVE_CH1_PIN, Bit_RESET);
#define DRIVE_TIM_CH1N_PULSE(X) 					TIM1->CCR1 = X
#define DRIVE_TIM_CH2_PULSE(X) 						X ? GPIO_WriteBit(DRIVE_PORT, DRIVE_CH2_PIN, Bit_SET) : GPIO_WriteBit(DRIVE_PORT, DRIVE_CH2_PIN, Bit_RESET);
#define DRIVE_TIM_CH2N_PULSE(X) 					TIM1->CCR2 = X
#define DRIVE_PORTSOURCE 									GPIO_PortSourceGPIOB
#define DRIVE_SIN_COS_PINSOURCE 					GPIO_PinSource11
#define DRIVE_OPEN_PINSOURCE 							GPIO_PinSource7
#define DRIVE_CLOSE_PINSOURCE 						GPIO_PinSource6
#define DRIVE_SIN_COS_EXTI_LINE 					EXTI_Line11
#define DRIVE_OPEN_EXTI_LINE 							EXTI_Line7
#define DRIVE_CLOSE_EXTI_LINE 						EXTI_Line6
#define DRIVE_SIN_COS_NVIC_IRQCHANNEL 		EXTI15_10_IRQn
#define DRIVE_SIN_COS_NVIC_IRQHANDLER 		EXTI15_10_IRQHandler
#define DRIVE_OPEN_CLOSE_NVIC_IRQCHANNEL 	EXTI9_5_IRQn
#define DRIVE_OPEN_CLOSE_NVIC_IRQHANDLER 	EXTI9_5_IRQHandler

#define HOLL_SIN_PIN	 										GPIO_Pin_11
#define HOLL_COS_PIN 											GPIO_Pin_10
#define HOLL_PORT 												GPIOB

#define TEST_PIN		 											GPIO_Pin_8
#define TEST_PORT 												GPIOA

#define DRIVE_COS_STATE 									GPIO_ReadInputDataBit(DRIVE_PORT, DRIVE_COS_PIN)
#define DRIVE_OPEN_TERMINAL_STATE 				GPIO_ReadInputDataBit(DRIVE_PORT, DRIVE_OPEN_PIN)
#define DRIVE_CLOSE_TERMINAL_STATE 				GPIO_ReadInputDataBit(DRIVE_PORT, DRIVE_CLOSE_PIN)

#define DRIVE_DIRECTION_FOREWARD 								1
#define DRIVE_DIRECTION_BACKWARD 								0

int switchInit( void );
int hollInit( void );
int driveInit( void );
void driveStart(uint8_t direction, uint16_t speed);
void driveStop(void);
//bool driveValveOpen(void);
//bool driveValveClose(void);
float driveGetAngle(void);
void driveSetAngle(float Angle);

void driveProcess(void);

#endif /* DRIVE_H_ */
