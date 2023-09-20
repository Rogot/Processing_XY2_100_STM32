#include "dma_cmsis.h"

/*
 *	@brief This function init DMA
*/

void CMSIS_DMA_Init(DMA_Stream_TypeDef* dma_stream){

	/* Recording data from USART_DWIN BEGIN */
	

	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN; //Enable DMA2

	if (dma_stream == DMA2_Stream2) {	
		dma_stream->CR |= DMA_SxCR_CHSEL_2; //Channel 4
		dma_stream->CR &= ~DMA_SxCR_DIR; //Peripheral-to-memory
		dma_stream->CR &= ~DMA_SxCR_CIRC; //Circular mode disable
		dma_stream->CR |= DMA_SxCR_PL; //Priority level very high
		dma_stream->CR &= ~DMA_SxCR_MSIZE; // 8 bit
		dma_stream->CR &= ~DMA_SxCR_PSIZE; // 8 bit
		dma_stream->CR |= DMA_SxCR_MINC; //Memory increment mode enable
		dma_stream->CR &= ~DMA_SxCR_PINC; //Peripheral increment mode disable
		dma_stream->CR |= DMA_SxCR_TCIE; //Interrupt enable
		dma_stream->CR &= ~DMA_SxCR_HTIE; //Interrupt half disable
	} else if (DMA2_Stream7) {
	dma_stream->CR |= DMA_SxCR_CHSEL_2; //Channel 4
		dma_stream->CR |= DMA_SxCR_DIR_0; //Memory-to-peripherial
		dma_stream->CR &= ~DMA_SxCR_CIRC; //Circular mode disable
		dma_stream->CR |= DMA_SxCR_PL; //Priority level very high
		dma_stream->CR &= ~DMA_SxCR_MSIZE; // 8 bit
		dma_stream->CR &= ~DMA_SxCR_PSIZE; // 8 bit
		dma_stream->CR |= DMA_SxCR_MINC; //Memory increment mode enable
		dma_stream->CR &= ~DMA_SxCR_PINC; //Peripheral increment mode disable
		dma_stream->CR |= DMA_SxCR_TCIE; //Interrupt enable
		dma_stream->CR &= ~DMA_SxCR_HTIE; //Interrupt half disable	
	}
	
	NVIC_EnableIRQ(DMA2_Stream2_IRQn);
	NVIC_EnableIRQ(DMA2_Stream7_IRQn);
	/* Recording data from USART_DWIN END */
	
}

void CMSIS_DMA_Config(DMA_Stream_TypeDef* dma_stream, uint32_t* srcAdrr, uint32_t* dstAdrr, uint16_t dataSize){
	//dma_stream->CR &= ~DMA_SxCR_EN; //Stream enable
	
	dma_stream->PAR = (uint32_t) srcAdrr;

	dma_stream->M0AR = (uint32_t) dstAdrr;

	dma_stream->NDTR = dataSize; //Buffer size

	dma_stream->CR |= DMA_SxCR_EN; //Stream enable
}