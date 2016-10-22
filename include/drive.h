/*
 * drive.h
 *
 *  Created on: 04 окт. 2016 г.
 *      Author: Gennady Tanchin <g.tanchin@yandex.ru>
 */

#ifndef DRIVE_H_
#define DRIVE_H_

#include "stm32f10x.h"
#include "can.h"


typedef enum {
	QUART_SPEED	= 25,
	HALF_SPEED = 50,
	FULL_SPEED = 100
} eSpeed;

typedef enum {
	ERR_OK,
	ERR_SWITCH,
	ERR_DRIVE,
	ERR_HOLL
} eValveErr;

typedef enum {
	DIR_STOP,				// Стоит
	DIR_FOREWARD,		// Открывается
	DIR_BACKWARD,		// Закрывается
} eDirect;

typedef enum {
	STATE_WORK,
	STATE_NEED_STOP,
	STATE_CORRECT,			// Стоит в правильном положении
	STATE_CALIBRATE,
} eState;

typedef enum {
	SW_NON,
	SW_OPEN,
	SW_CLOSE
} eSwitch;

typedef struct {
	eColdHot coldHot;
	uint8_t curDeg;
	uint8_t adjDeg;
	int32_t hollCount;
	eState state;
	eSwitch sw;
	eDirect dir;
	eValveErr err;
} tValve;

extern volatile tValve valve;

#define DRIVE_RCC_GPIO 										RCC_APB2Periph_GPIOB
#define DRIVE_CH1_PIN 										GPIO_Pin_15
#define DRIVE_CH1N_PIN 										GPIO_Pin_13
#define DRIVE_CH2_PIN 										GPIO_Pin_12
#define DRIVE_CH2N_PIN 										GPIO_Pin_14
#define DRIVE_PORT 												GPIOB
#define DRIVE_RCC_TIM 										RCC_APB2Periph_TIM1
#define DRIVE_TIM 												TIM1
#define DRIVE_TIM_CH1_PULSE(X) 						X ? GPIO_WriteBit(DRIVE_PORT, DRIVE_CH1_PIN, Bit_SET) : GPIO_WriteBit(DRIVE_PORT, DRIVE_CH1_PIN, Bit_RESET);
#define DRIVE_TIM_CH1N_PULSE(X) 					TIM1->CCR1 = X
#define DRIVE_TIM_CH2_PULSE(X) 						X ? GPIO_WriteBit(DRIVE_PORT, DRIVE_CH2_PIN, Bit_SET) : GPIO_WriteBit(DRIVE_PORT, DRIVE_CH2_PIN, Bit_RESET);
#define DRIVE_TIM_CH2N_PULSE(X) 					TIM1->CCR2 = X
#define DRIVE_PORT_CLK_ENABLE							RCC->APB2ENR |= RCC_APB2ENR_IOPBEN

#define SWITCH_OPEN_PIN 									GPIO_Pin_7
#define SWITCH_OPEN_PORT 									GPIOB
#define SWITCH_OPEN_PINSOURCE 						GPIO_PinSource7
#define SWITCH_OPEN_PORTSOURCE						GPIO_PortSourceGPIOB
#define SWITCH_OPEN_EXTI_LINE							EXTI_Line7
#define SWITCH_OPEN_PORT_CLK_ENABLE				RCC->APB2ENR |= RCC_APB2ENR_IOPBEN

#define SWITCH_CLOSE_PIN 									GPIO_Pin_6
#define SWITCH_CLOSE_PORT 								GPIOB
#define SWITCH_CLOSE_PINSOURCE 						GPIO_PinSource6
#define SWITCH_CLOSE_PORTSOURCE						GPIO_PortSourceGPIOB
#define SWITCH_CLOSE_EXTI_LINE 						EXTI_Line6
#define SWITCH_CLOSE_PORT_CLK_ENABLE			RCC->APB2ENR |= RCC_APB2ENR_IOPBEN

#define SWITCH_IRQCHANNEL 								EXTI9_5_IRQn
#define SWITCH_IRQHANDLER 								EXTI9_5_IRQHandler

#define HOLL_SIN_PIN	 										GPIO_Pin_11
#define HOLL_COS_PIN 											GPIO_Pin_10
#define HOLL_PORT 												GPIOB
#define HOLL_IRQCHANNEL			 							EXTI15_10_IRQn
#define HOLL_IRQHANDLER 									EXTI15_10_IRQHandler
#define HOLL_PORTSOURCE 									GPIO_PortSourceGPIOB
#define HOLL_PINSOURCE 										GPIO_PinSource11
#define HOLL_EXTI_LINE 										EXTI_Line11
#define HOLL_PORT_CLK_ENABLE							RCC->APB2ENR |= RCC_APB2ENR_IOPBEN

#define TEST_PIN		 											GPIO_Pin_8
#define TEST_PORT 												GPIOA

#define DRIVE_COS_STATE 									GPIO_ReadInputDataBit(DRIVE_PORT, DRIVE_COS_PIN)
#define DRIVE_OPEN_TERMINAL_STATE 				GPIO_ReadInputDataBit(DRIVE_PORT, DRIVE_OPEN_PIN)
#define DRIVE_CLOSE_TERMINAL_STATE 				GPIO_ReadInputDataBit(DRIVE_PORT, DRIVE_CLOSE_PIN)

extern volatile tValve valve;

int valveInit( void );
int switchInit( void );
int hollInit( void );
int driveInit( void );
void driveStart(uint8_t direction, eSpeed speed);
void driveStop(void);
//bool driveValveOpen(void);
//bool driveValveClose(void);
float driveGetAngle(void);
void driveSetAngle(float Angle);

int driveProcess(void);

#endif /* DRIVE_H_ */
