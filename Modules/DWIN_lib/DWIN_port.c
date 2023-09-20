/* Includes ------------------------------------------------------------------*/
#include "DWIN_port.h"
/* Declarations and definitions ----------------------------------------------*/

extern UART_HandleTypeDef* DWIN_uart;
extern DMA_Stream_TypeDef* DWIN_dma;
/* Functions -----------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/\
void DWIN_PORT_SetUartModule(UART_HandleTypeDef* uart)
{
  DWIN_uart = uart;
}
/*----------------------------------------------------------------------------*/

void DWIN_PORT_SetDMAModule(DMA_Stream_TypeDef* dma) {
	DWIN_dma = dma;
}