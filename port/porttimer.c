/*
 * FreeModbus Libary: LPC214X Port
 * Copyright (C) 2007 Tiago Prado Lone <tiago@maxwellbohr.com.br>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: porttimer.c,v 1.1 2007/04/24 23:15:18 wolti Exp $
 */

/* ----------------------- Platform includes --------------------------------*/
#include "main.h"
#include "port.h"
#include "stm32f4xx_it.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/*	static functions --------------------------------------------------------*/
/*	static variables --------------------------------------------------------*/
uint32_t counter_val = 0;
/*	exported variables ------------------------------------------------------*/
extern TIM_HandleTypeDef TimHandle;

/* ----------------------- Start implementation -----------------------------*/
BOOL
xMBPortTimersInit( USHORT usTim1Timerout50us )
{
    return TRUE;
}


void
vMBPortTimersEnable(  )
{
	HAL_TIM_Base_Stop_IT(&TimHandle);	// Disable Timer and Reset Counter
	__HAL_TIM_SET_COUNTER(&TimHandle, 0);
	
	HAL_TIM_Base_Start_IT(&TimHandle);	// Enable Timer
}

void
vMBPortTimersDisable(  )
{
	HAL_TIM_Base_Stop_IT(&TimHandle);		// Disable Timer and Reset Counter
}

/**
  * @brief  This function handles TIM interrupt request.
  * @param  None
  * @retval None
  */
void TIMx_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&TimHandle);
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_12);
	//counter_val = __HAL_TIM_GET_COUNTER(&TimHandle);
	//printf("Counter at interrupt: %d\n", counter_val);
	pxMBPortCBTimerExpired();
}
