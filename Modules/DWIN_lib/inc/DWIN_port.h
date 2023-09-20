#ifndef DWIN_PORT_H
#define DWIN_PORT_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
/* Declarations and definitions ----------------------------------------------*/

#define FUNC_CODE_READ					0x83
#define FUNC_CODE_WRITE					0x82

typedef enum
{
    DWIN_EX_NONE = 0x00,
    DWIN_EX_ILLEGAL_FUNCTION = 0x01,
    DWIN_EX_ILLEGAL_DATA_ADDRESS = 0x02,
    DWIN_EX_ILLEGAL_DATA_VALUE = 0x03,
    DWIN_EX_SLAVE_DEVICE_FAILURE = 0x04,
    DWIN_EX_ACKNOWLEDGE = 0x05,
    DWIN_EX_SLAVE_BUSY = 0x06,
    DWIN_EX_MEMORY_PARITY_ERROR = 0x08,
    DWIN_EX_GATEWAY_PATH_FAILED = 0x0A,
    DWIN_EX_GATEWAY_TGT_FAILED = 0x0B
} eDWINException;

/* Functions -----------------------------------------------------------------*/
void DWIN_PORT_SetDMAModule(DMA_Stream_TypeDef* dma);
void DWIN_PORT_SetUartModule(UART_HandleTypeDef* uart);
#endif // #DWIN_PORT_H