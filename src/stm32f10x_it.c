/**
  ******************************************************************************
  * @file    stm32f2xx_it.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    07-October-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <flow.h>
#include <main.h>
#include "stm32f10x_it.h"
#include "my_time.h"
#include "can.h"
#include "drive.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

// ----- SysTick_Handler() ----------------------------------------------------

void
SysTick_Handler (void) {
#if defined(USE_HAL_DRIVER)
  HAL_IncTick();
#endif
  myTick++;
  timersHandler();
}

/**
  * @brief  This function handles External line 3 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI3_IRQHandler(void){
}

/******************************************************************************/
/*                 STM32F2xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f2xx.s).                                               */
/******************************************************************************/
// Прерывание датчика Холла - измерителя потока
void EXTI9_5_IRQHandler(void){
	SWITCH_IRQHandler();
}

/**
  * @brief  This function handles External line 10 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI15_10_IRQHandler(void){
	if (EXTI_GetITStatus(FLOW_SENS_EXTI_LINE)) {
		HOLL_IRQHandler();
		EXTI_ClearITPendingBit(FLOW_SENS_EXTI_LINE);
	}
}

void TIM4_IRQHandler( void ){
	valve.sw = SW_NON;
	if( (SWITCH_OPEN_PORT->IDR & SWITCH_OPEN_PIN) == 0 ){
		valve.sw = SW_OPEN;
	}
	else if( (SWITCH_CLOSE_PORT->IDR & SWITCH_CLOSE_PIN) == 0 ){
		valve.sw = SW_CLOSE;
	}
	DEBOUNCE_TIM->CR1 &= ~TIM_CR1_CEN;
	DEBOUNCE_TIM->SR &= ~TIM_SR_UIF;
}


void RTC_IRQHandler(void){
	if( RTC->CRL & RTC_CRL_SECF ){
		uxTime++;
		sysRtc.SecFlag = SET;
		RTC->CRL &= ~RTC_CRL_SECF;
		// Секундный таймер
		utime2Tm( &sysRtc, uxTime );
			// Выставляем флаги минут, часов, дней, недель и месяцев
		if( sysRtc.sec == 0 ){
			sysRtc.MinFlag = SET;
			if( sysRtc.min == 0){
				sysRtc.HourFlag = SET;
				if( sysRtc.hour == 0){
					sysRtc.DayFlag = SET;
					if( sysRtc.wday == 1){
						sysRtc.WeekFlag = SET;
					}
					if( sysRtc.mday == 1){
						sysRtc.MonthFlag = SET;
					}
				}
			}
		}
	}
	if( RTC->CRL & RTC_CRL_ALRF ){
		RTC->CRL &= ~RTC_CRL_ALRF;
	}
	// Переполнение счетного регистра
	if(RTC->CRL & RTC_CRL_OWF) {
     RTC->CRL &= ~RTC_CRL_OWF;     //сбросить флаг (обязательно!!!)
     //выполняем какие-то действия
	}
}


void USB_LP_CAN1_RX0_IRQHandler(void)
{
	canRx0IrqHandler();
}

void USB_HP_CAN1_TX_IRQHandler( void ){
	canTxIrqHandler();
}

void CAN1_RX1_IRQHandler( void ){
	canRx1IrqHandler();
}

void CAN1_SCE_IRQHandler( void ){
	canSceIrqHandler();
}

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
