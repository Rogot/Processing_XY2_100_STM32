#ifndef DMA_CMSIS_H
#define DMA_CMSIS_H

#include <stm32f405xx.h>
#include "stm32f4xx_hal.h"

#define DMA_ENABLE							( 1 )

void CMSIS_DMA_Init(DMA_Stream_TypeDef* dma_stream);
void CMSIS_DMA_Config(DMA_Stream_TypeDef* dma_stream, 
											uint32_t* srcAdrr, uint32_t* dstAdrr, 
											uint16_t dataSize);

#endif //!DMA_CMSIS_H