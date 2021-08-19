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
 * File: $Id: portevent.c,v 1.60 2013/08/13 15:07:05 Armink $
 */

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- Variables ----------------------------------------*/
//static struct rt_event     xSlaveOsEvent;
//static EventGroupHandle_t  xSlaveOsEvent;

osEventFlagsId_t myEvent02Handle;
const osEventFlagsAttr_t myEvent02_attributes = {
  .name = "myEvent02"
};

/* ----------------------- Start implementation -----------------------------*/
BOOL xMBPortEventInit( void )
{
//    rt_event_init(&xSlaveOsEvent,"slave event",RT_IPC_FLAG_PRIO);
    myEvent02Handle = osEventFlagsNew(&myEvent02_attributes);
    return TRUE;
}

BOOL xMBPortEventPost( eMBEventType eEvent )
{
//    rt_event_send(&xSlaveOsEvent, eEvent);
    osEventFlagsSet( myEvent02Handle,eEvent );
    return TRUE;
}

BOOL xMBPortEventGet( eMBEventType * eEvent )
{
    uint32_t recvedEvent;
    /* waiting forever OS event */
//    rt_event_recv(&xSlaveOsEvent,
//            EV_READY | EV_FRAME_RECEIVED | EV_EXECUTE | EV_FRAME_SENT,
//            RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER,
//            &recvedEvent);
    
    recvedEvent = osEventFlagsWait( myEvent02Handle,
                                   EV_READY | EV_FRAME_RECEIVED | EV_EXECUTE | EV_FRAME_SENT,
                                   osFlagsWaitAny,
                                   osWaitForever
                                   );
    
    switch (recvedEvent)
    {
    case EV_READY:
        *eEvent = EV_READY;
        break;
    case EV_FRAME_RECEIVED:
        *eEvent = EV_FRAME_RECEIVED;
        break;
    case EV_EXECUTE:
        *eEvent = EV_EXECUTE;
        break;
    case EV_FRAME_SENT:
        *eEvent = EV_FRAME_SENT;
        break;
    }
    return TRUE;
}
