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
 * File: $Id: portserial.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

#include "port.h"
#include "usart.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "mbrtu.h"

/* ----------------------- static functions ---------------------------------*/
UART_HandleTypeDef* huart;
uint8_t gRxBuffer;
/* ----------------------- Start implementation -----------------------------*/



void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
    /* If xRXEnable enable serial receive interrupts. If xTxENable enable
     * transmitter empty interrupts.
     */
    HAL_UART_StateTypeDef state = HAL_UART_GetState(huart);
    dbg_printf("HAL_UART_State = 0x%x\n",(int)state);

	if (xRxEnable)
    {
        dbg_printf("RxEnable \n");
        		//HAL_GPIO_WritePin(DE_Port, DE_Pin, GPIO_PIN_RESET);
		//__HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);

		HAL_UART_Receive_IT(huart, &gRxBuffer, 1);
	}
	else
	{
	    dbg_printf("RxDisable ");
        switch(state)
        {
        case HAL_UART_STATE_ERROR:
            HAL_UART_Abort(huart);
        case HAL_UART_STATE_BUSY_RX:
        case HAL_UART_STATE_BUSY_TX_RX:
            HAL_UART_AbortReceive_IT(huart);
            //HAL_UART_Abort(huart);
        default:
            __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);
            break;
        }
        dbg_printf("\n");
	}


	if (xTxEnable)
    {
        dbg_printf("TxEnable \n");
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(DE_Port, DE_Pin, GPIO_PIN_SET);
		//__HAL_UART_ENABLE_IT(huart, UART_IT_TXE);
		HAL_UART_TxCpltCallback(huart);
	}
	else
	{
	    dbg_printf("TxDisable ");
	    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_SET);

		switch(state)
        {
        case HAL_UART_STATE_ERROR:
            HAL_UART_Abort(huart);
        case HAL_UART_STATE_BUSY_TX:
        case HAL_UART_STATE_BUSY_TX_RX:
            HAL_UART_AbortTransmit_IT(huart);

        default:
            __HAL_UART_DISABLE_IT(huart, UART_IT_TXE);
            break;
        }
        dbg_printf("\n");
	}


}

BOOL
xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
    UNUSED(ucPORT);
    UNUSED(ulBaudRate);
    UNUSED(ucDataBits);
    UNUSED(eParity);

    huart = &huart1;
  	MX_USART1_UART_Init();
    return TRUE;
}

BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
    /* Put a byte in the UARTs transmit buffer. This function is called
     * by the protocol stack if pxMBFrameCBTransmitterEmpty( ) has been
     * called. */
	//huart->Instance->DR = ucByte;
    gRxBuffer=ucByte;
	HAL_UART_Transmit_IT(huart, &gRxBuffer, 1);
    dbg_printf("Tx>: %x \r\n", ucByte);
    return TRUE;
}
//-----------------------------------------------------------------------------
inline BOOL xMBPortSerialGetByte( CHAR * pucByte )
{
    /* Return the byte in the UARTs receive buffer. This function is called
     * by the protocol stack after pxMBFrameCBByteReceived( ) has been called.
     */
    HAL_UART_Receive_IT(&huart1, &gRxBuffer, 1);
    *pucByte = gRxBuffer;
    //dbg_printf("Rx>: %x \r\n", *pucByte);
    return TRUE;
}

/* Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call
 * xMBPortSerialPutByte( ) to send the character.
 */
//void prvvUARTTxReadyISR( void )
//{
   //pxMBFrameCBTransmitterEmpty(  );
//}

/* Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.
 */
//void prvvUARTRxISR( void )
//{
    //pxMBFrameCBByteReceived(  );
//}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
        pxMBFrameCBTransmitterEmpty();
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
        pxMBFrameCBByteReceived();
}

/*
void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    dbg_printf("HAL_UART_TxHalfCpltCallback\n");
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    dbg_printf("HAL_UART_RxHalfCpltCallback\n");
}
*/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    //#define HAL_UART_ERROR_NONE         0x00000000U   /*!< No error            */
    //#define HAL_UART_ERROR_PE           0x00000001U   /*!< Parity error        */
    //#define HAL_UART_ERROR_NE           0x00000002U   /*!< Noise error         */
    //#define HAL_UART_ERROR_FE           0x00000004U   /*!< Frame error         */
    //#define HAL_UART_ERROR_ORE          0x00000008U   /*!< Overrun error       */
    //#define HAL_UART_ERROR_DMA          0x00000010U   /*!< DMA transfer error  */
    uint32_t error = HAL_UART_GetError(huart);
    dbg_printf("HAL_UART_ErrorCallback = 0x%x\n", (unsigned int)HAL_UART_GetError(huart));
    vMBPortTimersDisable();
    eMBRTUStop();
    //__HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);
    //__HAL_UART_DISABLE_IT(huart, UART_IT_TXE);
    HAL_UART_Abort(huart);
    while(error)
    {
        switch(error)
        {
        case HAL_UART_ERROR_NONE:
        default: break;
        case HAL_UART_ERROR_PE:__HAL_UART_CLEAR_PEFLAG(huart);error&=~HAL_UART_ERROR_PE;break;
        case HAL_UART_ERROR_NE:__HAL_UART_CLEAR_NEFLAG(huart);error&=~HAL_UART_ERROR_NE;break;
        case HAL_UART_ERROR_FE:__HAL_UART_CLEAR_FEFLAG(huart);error&=~HAL_UART_ERROR_FE;break;
        case HAL_UART_ERROR_ORE:__HAL_UART_CLEAR_OREFLAG(huart);error&=~HAL_UART_ERROR_ORE;break;
        case HAL_UART_ERROR_DMA:        HAL_UART_DMAStop(huart);error&=~HAL_UART_ERROR_DMA;break;
        }//switch(error)
    }


    //ENTER_CRITICAL_SECTION();
    //eMBRTUStop();
    //__HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);
    //__HAL_UART_DISABLE_IT(huart, UART_IT_TXE);
    //HAL_UART_AbortTransmit_IT(huart);
    //HAL_UART_AbortReceive_IT(huart);

    //HAL_UART_Abort(huart);
    //huart->ErrorCode=HAL_UART_ERROR_NONE;
    //dbg_printf("UART state - %x\n", HAL_UART_GetState(huart));
    //EXIT_CRITICAL_SECTION();
    //HAL_UART_MspDeInit(huart);
    //HAL_UART_MspInit(huart);
    //MX_USART1_UART_Init();

    eMBRTUStart();
    //NVIC_SystemReset ();
}
void HAL_UART_AbortCpltCallback (UART_HandleTypeDef *huart)
{
    UNUSED(huart);
    dbg_printf("HAL_UART_AbortCpltCallback\n");
}
void HAL_UART_AbortTransmitCpltCallback (UART_HandleTypeDef *huart)
{
    UNUSED(huart);
    dbg_printf("HAL_UART_AbortTransmitCpltCallback\n");
}
void HAL_UART_AbortReceiveCpltCallback (UART_HandleTypeDef *huart)
{
    UNUSED(huart);
    dbg_printf("HAL_UART_AbortReceiveCpltCallback\n");
}
