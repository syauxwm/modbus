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
 * File: $Id: portserial.c,v 1.60 2013/08/13 15:07:05 Armink $
 */

#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- Static variables ---------------------------------*/
/* software simulation serial transmit IRQ handler thread */
static TaskHandle_t thread_serial_soft_trans_irq = NULL;
/* serial event */
//static EventGroupHandle_t event_serial;
/* modbus slave serial device */
static UART_HandleTypeDef *serial;
/*
 * Serial FIFO mode
 */
/* Definitions for myTask02 */
osThreadId_t myTaskSlaveHandle;
const osThreadAttr_t myTaskSlave_attributes = {
  .name = "myTaskSlave",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};

osEventFlagsId_t myEvent03Handle;
const osEventFlagsAttr_t myEvent03_attributes = {
  .name = "myEvent03"
};

static volatile uint8_t rx_buff[FIFO_SIZE_MAX];
static Serial_fifo Slave_serial_rx_fifo;
/* ----------------------- Defines ------------------------------------------*/
/* serial transmit event */
#define EVENT_SERIAL_TRANS_START    (1<<0)

/* ----------------------- static functions ---------------------------------*/
static void prvvUARTTxReadyISR(void);
static void prvvUARTRxISR(void);
static void serial_soft_trans_irq(void *parameter);
static void Slave_TxCpltCallback(struct __UART_HandleTypeDef *huart);
static void Slave_RxCpltCallback(struct __UART_HandleTypeDef *huart);
static int stm32_getc(void);
static int stm32_putc(CHAR c);

/* ----------------------- Start implementation -----------------------------*/
BOOL xMBPortSerialInit(UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits,
        eMBParity eParity)
{
   
    char uart_name[20];
    /**
     * set 485 mode receive and transmit control IO
     * @note MODBUS_SLAVE_RT_CONTROL_PIN_INDEX need be defined by user
     */
//#if defined(RT_MODBUS_SLAVE_USE_CONTROL_PIN)
//    rt_pin_mode(MODBUS_SLAVE_RT_CONTROL_PIN_INDEX, PIN_MODE_OUTPUT);
//#endif
    /* set serial name */
  /* set serial name */
  if (ucPORT == 1) {
#if defined(USING_UART1)
    extern UART_HandleTypeDef huart1;
    serial = &huart1;
//    MODBUS_DEBUG("Slave using uart1!\r\n");

#endif
  } else if (ucPORT == 2) {
#if defined(USING_UART2)
    extern UART_HandleTypeDef huart2;
    serial = &huart2;
//    MODBUS_DEBUG("Slave using uart2!\r\n");

#endif
  } else if (ucPORT == 3) {
#if defined(USING_UART3)
    extern UART_HandleTypeDef huart3;
    serial = &huart3;
//    MODBUS_DEBUG("Slave using uart3!\r\n");
#endif
  }

    /* set serial configure parameter */
    serial->Init.StopBits = UART_STOPBITS_1;
    serial->Init.BaudRate = ulBaudRate;
    switch (eParity) {
    case MB_PAR_NONE: {
      serial->Init.WordLength = UART_WORDLENGTH_8B;
      serial->Init.Parity = UART_PARITY_NONE;
      break;
    }
    case MB_PAR_ODD: {
      serial->Init.WordLength = UART_WORDLENGTH_9B;
      serial->Init.Parity = UART_PARITY_ODD;
      break;
    }
    case MB_PAR_EVEN: {
      serial->Init.WordLength = UART_WORDLENGTH_9B;
      serial->Init.Parity = UART_PARITY_EVEN;
      break;
    }
    }
    if (HAL_UART_Init(serial) != HAL_OK) {
      Error_Handler();
    }
    
  __HAL_UART_DISABLE_IT(serial, UART_IT_RXNE);
  __HAL_UART_DISABLE_IT(serial, UART_IT_TC);
  /*registe recieve callback*/
  HAL_UART_RegisterCallback(serial, HAL_UART_RX_COMPLETE_CB_ID,
                            Slave_RxCpltCallback);
  /* software initialize */
  Slave_serial_rx_fifo.buffer = rx_buff;
  Slave_serial_rx_fifo.get_index = 0;
  Slave_serial_rx_fifo.put_index = 0;


    /* software initialize */
//    rt_event_init(&event_serial, "slave event", RT_IPC_FLAG_PRIO);
    
    
//    rt_thread_init(&thread_serial_soft_trans_irq,
//                   "slave trans",
//                   serial_soft_trans_irq,
//                   RT_NULL,
//                   serial_soft_trans_irq_stack,
//                   sizeof(serial_soft_trans_irq_stack),
//                   10, 5);
//    rt_thread_startup(&thread_serial_soft_trans_irq);

    myEvent03Handle = osEventFlagsNew(&myEvent03_attributes);            //创建事件
    myTaskSlaveHandle = osThreadNew(serial_soft_trans_irq, NULL, &myTaskSlave_attributes);
    return TRUE;
}

void vMBPortSerialEnable(BOOL xRxEnable, BOOL xTxEnable)
{
//    rt_uint32_t recved_event;
    
   __HAL_UART_CLEAR_FLAG(serial,UART_FLAG_RXNE);
   __HAL_UART_CLEAR_FLAG(serial,UART_FLAG_TC);
   
    if (xRxEnable)
    {
      /* enable RX interrupt */
      __HAL_UART_ENABLE_IT(serial, UART_IT_RXNE);
      /* switch 485 to receive mode */
//      MODBUS_DEBUG("RS485_RX_MODE\r\n");
      SLAVE_RS485_RX_MODE;
    }
    else
    {
      /* switch 485 to transmit mode */
//      MODBUS_DEBUG("RS485_TX_MODE\r\n");
      SLAVE_RS485_TX_MODE;
      /* disable RX interrupt */
      __HAL_UART_DISABLE_IT(serial, UART_IT_RXNE);
    }
    if (xTxEnable)
    {
        /* start serial transmit */
//        rt_event_send(&event_serial, EVENT_SERIAL_TRANS_START);
        osEventFlagsSet( myEvent03Handle,EVENT_SERIAL_TRANS_START );
    }
    else
    {
        /* stop serial transmit */
//        rt_event_recv(&event_serial, EVENT_SERIAL_TRANS_START,
//                RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 0,
//                &recved_event);
        
        
        osEventFlagsClear( myEvent03Handle,EVENT_SERIAL_TRANS_START );
    }
}

void vMBPortClose(void)
{
//    serial->parent.close(&(serial->parent));
  __HAL_UART_DISABLE(serial); 
}

BOOL xMBPortSerialPutByte(CHAR ucByte)
{
//    serial->parent.write(&(serial->parent), 0, &ucByte, 1);
    stm32_putc(ucByte);
    return TRUE;
}

BOOL xMBPortSerialGetByte(CHAR * pucByte)
{
//    serial->parent.read(&(serial->parent), 0, pucByte, 1);
    Get_from_fifo(&Slave_serial_rx_fifo, (uint8_t *)pucByte, 1);
    return TRUE;
}

/* 
 * Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call 
 * xMBPortSerialPutByte( ) to send the character.
 */
void prvvUARTTxReadyISR(void)
{
    pxMBFrameCBTransmitterEmpty();
}

/* 
 * Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.
 */
void prvvUARTRxISR(void)
{
    pxMBFrameCBByteReceived();
}

/**
 * Software simulation serial transmit IRQ handler.
 *
 * @param parameter parameter
 */
static void serial_soft_trans_irq(void* parameter) {
    uint32_t recved_event;
    while (1)
    {
        /* waiting for serial transmit start */
//        rt_event_recv(&event_serial, EVENT_SERIAL_TRANS_START, RT_EVENT_FLAG_OR,
//                RT_WAITING_FOREVER, &recved_event);
        
        recved_event = osEventFlagsWait( myEvent03Handle,
                                       EVENT_SERIAL_TRANS_START,
                                       osFlagsNoClear,
                                       osWaitForever
                                        );        
        
        /* execute modbus callback */
        prvvUARTTxReadyISR();
    }
}

/**
 * This function is serial receive callback function
 *
 * @param dev the device of serial
 * @param size the data size that receive
 *
 * @return return RT_EOK
 */
void Slave_RxCpltCallback(UART_HandleTypeDef *huart) {
  int ch = -1;
  while (1) {
    ch = stm32_getc();
    if (ch == -1)
      break;
    Put_in_fifo(&Slave_serial_rx_fifo, (uint8_t *)&ch, 1);
  }
  prvvUARTRxISR();
}
/*UART发送一个字节*/
static int stm32_putc(CHAR c) {
  serial->Instance->DR = c;
  while (!(serial->Instance->SR & UART_FLAG_TC))
    ;
  return TRUE;
}
/*UART接收一个字节*/
static int stm32_getc(void) {
  int ch;
  ch = -1;
  if (serial->Instance->SR & UART_FLAG_RXNE) {
    ch = serial->Instance->DR & 0xff;
  }
  return ch;
}
