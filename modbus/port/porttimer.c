/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
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
 * File: $Id: porttimer.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"
#include "tim.h"
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"


/* ----------------------- static functions ---------------------------------*/
TIM_HandleTypeDef* htim;
//uint16_t timeout = 0;
//volatile uint16_t downcounter  = 0;


/* ----------------------- Start implementation -----------------------------*/
BOOL xMBPortTimersInit( USHORT usTim1Timerout50us )
{
    UNUSED(usTim1Timerout50us);
    //vMBPortTimersDisable();
    //timeout = usTim1Timerout50us;
    htim = &htim4;
    MX_TIM4_Init();
  return TRUE;
    /*
    htim.Instance = TIM4;
	htim.Init.CounterMode = TIM_COUNTERMODE_UP;

	htim.Init.Prescaler = (HAL_RCC_GetPCLK1Freq() / 1000000) - 1;
	htim.Init.Period = ( (uint32_t) usTim1Timerout50us ) - 1;
    //htim.Init.Prescaler = 7199;
    //htim.Init.Period = 500;

	timeout = usTim1Timerout50us;

    return HAL_OK == HAL_TIM_Base_Init(&htim) ? TRUE : FALSE;
    */
}
//-----------------------------------------------------------------------------

inline void vMBPortTimersEnable(  )
{
    /* Enable the timer with the timeout passed to xMBPortTimersInit( ) */
    //downcounter = timeout;
    HAL_TIM_Base_Start_IT(htim);
}
//-----------------------------------------------------------------------------
inline void vMBPortTimersDisable(  )
{
   /* Disable any pending timers. */
    HAL_TIM_Base_Stop_IT(htim);
}
//-----------------------------------------------------------------------------
/* Create an ISR which is called whenever the timer has expired. This function
 * must then call pxMBPortCBTimerExpired( ) to notify the protocol stack that
 * the timer has expired.
 */
void TIM4PeriodElapsedCallback( void )
{
    //HAL_UART_DMAStop(&huart1);
    ( void )pxMBPortCBTimerExpired(  );
    dbg_printf("TimerExpired\r\n");
}
//-----------------------------------------------------------------------------

























