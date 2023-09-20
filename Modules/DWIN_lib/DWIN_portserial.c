#include "DWIN_lib.h"
#include "stm32f4xx_hal.h"
/* ----------------------- static functions ---------------------------------*/
static void UARTTxReadyISR( void );
static void UARTRxISR( void );

/* ----------------------- Variables ----------------------------------------*/
extern UART_HandleTypeDef* DWIN_uart;
extern DMA_Stream_TypeDef* DWIN_dma;
uint8_t DWIN_txByte = 0x00;
uint8_t DWIN_rxByte = 0x00;

BOOL xDWINPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eDWINParity eParity ) {
		/* Initilize some data */
}

void
vDWINPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
    /* If xRXEnable enable serial receive interrupts. If xTxENable enable
     * transmitter empty interrupts.
     */
		if (xRxEnable == FALSE)
		{
			HAL_UART_AbortReceive_IT(DWIN_uart);
		}
		else
		{
			HAL_UART_Receive_IT(DWIN_uart, &DWIN_rxByte, 1);
		}
		if (xTxEnable == FALSE)
		{
			HAL_UART_AbortTransmit_IT(DWIN_uart);
		}
		else
		{
			if (DWIN_uart->gState == HAL_UART_STATE_READY)
			{
				UARTTxReadyISR();
			}
		}
}

BOOL
xDWINPortSerialGetByte( CHAR * pucByte )
{
    /* Return the byte in the UARTs receive buffer. This function is called
     * by the protocol stack after pxMBFrameCBByteReceived( ) has been called.
     */
		
		*pucByte = DWIN_rxByte;
		HAL_UART_Receive_IT(DWIN_uart, &DWIN_rxByte, 1);
    return TRUE;
}

/* Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call 
 * xMBPortSerialPutByte( ) to send the character.
 */

static void UARTTxReadyISR( void )
{

}

/* Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.
 */
static void UARTRxISR( void )
{

}

/* --------------------------------------------------------------------------*/

#if DWIN_SERIAL_PORT_ENABLE
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == DWIN_uart) {
/*		eDWINEventType xEvent;
		xDWINPortEventGet(&xEvent);
		
		if (xEvent == DWIN_EV_EXECUTE) {
			xDWINPortEventPost(DWIN_EV_FRAME_SENT);
		}
		*/
		//DWIN_uart->Instance->SR &= ~USART_SR_RXNE;
	}
}
/* --------------------------------------------------------------------------*/


extern volatile UCHAR  ucDWINBuf[];

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
		if (huart->Instance == DWIN_uart->Instance)
		{
			eDWINEventType xEvent;
			xDWINSetQueue();
			xDWINPortEventGet(&xEvent);
			
			if (xEvent == DWIN_EV_READY) {
				xDWINPortEventPost(DWIN_EV_FRAME_RECEIVED);
			}
			
			//HAL_UART_Transmit_IT(DWIN_uart,(uint8_t *)ucDWINBuf, Size);
			
			/*HAL_UARTEx_ReceiveToIdle_IT(DWIN_uart,
															(uint8_t *)ucDWINBuf,
															DWIN_SER_PDU_SIZE_MAX);
			*/
			//CMSIS_DMA_Config(DWIN_dma, &(DWIN_uart->Instance->DR), (uint32_t*)ucDWINBuf, 3);
			//DWIN_uart->Instance->SR &= ~USART_SR_RXNE;
			//xDWINPortEventPost(DWIN_EV_FRAME_RECEIVED);
		}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	 if (huart->Instance == DWIN_uart->Instance)
		{
				//CMSIS_DMA_Config(DWIN_dma, &(DWIN_uart->Instance->DR), (uint32_t*)ucDWINBuf, 3);
				//DWIN_uart->Instance->SR &= ~USART_SR_RXNE;
				//xDWINPortEventPost(DWIN_EV_FRAME_RECEIVED);

		}
}

/* --------------------------------------------------------------------------*/
void DMA2_Stream2_IRQHandler(void)
{
	eDWINEventType xEvent;
	xDWINSetQueue();
	xDWINPortEventGet(&xEvent);
	
	if (xEvent == DWIN_EV_READY) {
		xDWINPortEventPost(DWIN_EV_FRAME_RECEIVED);
	} else if (xEvent == DWIN_EV_FRAME_RECEIVED) {
		xDWINPortEventPost(DWIN_EV_EXECUTE);
	}
	
	if (DMA2->LISR & DMA_LISR_TCIF2){
		DMA2->LIFCR |= DMA_LIFCR_CTCIF2;
		DMA2->LIFCR |= DMA_LIFCR_CHTIF2;
	}
}

extern DMA_HandleTypeDef hdma_usart1_tx;

void DMA2_Stream7_IRQHandler(void)
{
	eDWINEventType xEvent;
	//xDWINSetQueue();
	//xDWINPortEventGet(&xEvent);
	
	ucDWINBuf[0] = 0x99;
	
	//if (xEvent == DWIN_EV_EXECUTE) {
	xDWINPortEventPost(DWIN_EV_FRAME_SENT);
	//}
	#if 0
	if (DMA2->HISR & DMA_HISR_TCIF7){
		DMA2->HIFCR |= DMA_HIFCR_CTCIF7;
		DMA2->HIFCR |= DMA_HIFCR_CHTIF7;
	
		while (!(DWIN_uart->Instance->SR & USART_SR_TC));
	}
	#endif
	HAL_DMA_IRQHandler(&hdma_usart1_tx);
}

#endif
