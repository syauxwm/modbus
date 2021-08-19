/*
 * FreeModbus Libary: RT-Thread Port
 * Copyright (C) 2013 Armink <armink.ztl@gmail.com>
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
 * File: $Id: porttimer.c,v 1.60 2013/08/13 15:07:05 Armink $
 */

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- static functions ---------------------------------*/
//static struct rt_timer timer;
//static void prvvTIMERExpiredISR(void);
//static void timer_timeout_ind(void* parameter);
//
//
///* Definitions for myTimer01 */
//osTimerId_t myTimer02Handle;
//const osTimerAttr_t myTimer02_attributes = {
//  .name = "myTimer02"
//};
//uint32_t usTick = 0 ;


static TimerHandle_t timer;
static void prvvTIMERExpiredISR(void);
static void timer_timeout_ind(TimerHandle_t xTimer);

/* ----------------------- Start implementation -----------------------------*/
BOOL xMBPortTimersInit(USHORT usTim1Timerout50us)
{
  timer = xTimerCreate(
                      "Slave timer",
                      (50 * usTim1Timerout50us) / (1000 * 1000 / configTICK_RATE_HZ) + 1,
                      pdFALSE, 
                      (void *)2, 
                      timer_timeout_ind);
  if (timer != NULL)
    return TRUE;
  
  
//    uint32_t lll;
//    lll = 50*usTim1Timerout50us;
//    usTick = lll / (1000 * 1000 / configTICK_RATE_HZ) + 1;
//    
//    myTimer02Handle = osTimerNew( timer_timeout_ind, osTimerOnce, NULL, &myTimer02_attributes);
//
//    return TRUE;
}

void vMBPortTimersEnable()
{
//    rt_timer_start(&timer);
//    if( usTick >= 1 )
//      osTimerStart( myTimer02Handle,usTick);
   BaseType_t xHigherPriorityTaskWoken = pdFALSE;  
  if (IS_IRQ()) 
  {
    xTimerStartFromISR((TimerHandle_t)timer, 0);
  } else {
    xTimerStart((TimerHandle_t)timer, 0);
  }    
}

void vMBPortTimersDisable()
{
//    rt_timer_stop(&timer);
    
//    osTimerStop( myTimer02Handle );
    
  if (IS_IRQ()) {
    xTimerStopFromISR((TimerHandle_t)timer, 0);
  } else {
    xTimerStop((TimerHandle_t)timer, 0);
  }    
}

void prvvTIMERExpiredISR(void)
{
    (void) pxMBPortCBTimerExpired();
}

static void timer_timeout_ind(TimerHandle_t xTimer)
{
    prvvTIMERExpiredISR();
}
