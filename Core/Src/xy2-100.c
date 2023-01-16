#include "xy2-100.h"

//-----------------------------------------------------------------------------
// 	void CMSIS_GPIO_Init(void)
//	@brief This function init GPIOx
//-----------------------------------------------------------------------------

void CMSIS_GPIO_Init(void){
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; //Enable GPIOB
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //Enable GPIOA
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; //Enable GPIOC

	/* Временная замена информационных пинов */
	/*	PA10 - X	*/
	GPIOA->MODER &= ~GPIO_MODER_MODE10; // Input mode
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR10_1; //High speed

	/*	PA9 - Y	*/
	GPIOA->MODER &= ~GPIO_MODER_MODE9; // Input mode
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9_1; //High speed

	/*	PA6 - Z	*/
	GPIOA->MODER &= ~GPIO_MODER_MODE6; // Input mode
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6_1; //High speed

	/*	PA0 - CLCK*/
	GPIOA->MODER |= GPIO_MODER_MODE0_1; // Alternative mode
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR0_1; //High speed
	GPIOA->AFR[0] |= GPIO_AFRL_AFRL0_0 | GPIO_AFRL_AFRL0_1; //Alternate function for PA0 - AF3 (TIM8..TIM11)

	/*	PA2 - SYNC	*/
	GPIOA->MODER &= ~GPIO_MODER_MODE2; // Input mode
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR2_1; //High speed

	/* ~Временная замена информационных пинов~ */

	// PA1 - TIM2 input channel 2
	GPIOA->MODER &= ~GPIO_MODER_MODE1;
	GPIOA->MODER |= GPIO_MODER_MODE1_1; // Alternate function mode
	//GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR1_1; //High speed
	//GPIOA->PUPDR |= GPIO_PUPDR_PUPD1_1; //Pull-down
	GPIOA->AFR[0] |= GPIO_AFRL_AFRL1_0; //Alternate function for PA1 - AF1 (TIM1/TIM2)

	// PA3 - TIM2 input channel 4
	GPIOA->MODER &= ~GPIO_MODER_MODE3;
	GPIOA->MODER |= GPIO_MODER_MODE3_1; // Alternate function mode
	//GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR3_1; //High speed
	//GPIOA->PUPDR |= GPIO_PUPDR_PUPD3_1; //Pull-down
	GPIOA->AFR[0] |= GPIO_AFRL_AFRL3_0; //Alternate function for PA3 - AF1 (TIM1/TIM2)

}

//-----------------------------------------------------------------------------
// 	void CMSIS_DMA_Init(DMA_Stream_TypeDef* dma_stream)
//	@brief This function init DMA2 for working with TIM8
//-----------------------------------------------------------------------------

void CMSIS_DMA_Init(DMA_Stream_TypeDef* dma_stream){
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN; //Enable DMA2

	dma_stream->CR |= DMA_SxCR_CHSEL; //Channel 7
	dma_stream->CR &= ~DMA_SxCR_DIR; //Peripheral-to-memory
	dma_stream->CR |= DMA_SxCR_CIRC; //Circular mode enable
	dma_stream->CR |= DMA_SxCR_PL; //Priority level very high
	dma_stream->CR |= DMA_SxCR_MSIZE_0; // 16 bit
	dma_stream->CR |= DMA_SxCR_PSIZE_0; // 16 bit
	dma_stream->CR |= DMA_SxCR_MINC; //Memory increment mode enable
	dma_stream->CR &= ~DMA_SxCR_PINC; //Peripheral increment mode disable
	dma_stream->CR |= DMA_SxCR_TCIE; //Interrupt enable
	//dma_stream->CR |= DMA_SxCR_HTIE; //Interrupt half enable
}

//-----------------------------------------------------------------------------
// 	void CMSIS_DMA_Config(DMA_Stream_TypeDef* dma_stream, uint32_t srcAdrr, uint32_t dstAdrr, uint16_t dataSize)
//	@brief This function configure DMA
//-----------------------------------------------------------------------------

void CMSIS_DMA_Config(DMA_Stream_TypeDef* dma_stream, uint32_t srcAdrr, uint32_t dstAdrr, uint16_t dataSize){
	dma_stream->PAR = (uint32_t)srcAdrr;

	dma_stream->M0AR = (uint32_t)dstAdrr;

	dma_stream->NDTR = dataSize; //Buffer size

	dma_stream->CR |= DMA_SxCR_EN; //Stream enable

	//NVIC_EnableIRQ(DMA2_Stream2_IRQn);
	NVIC_EnableIRQ(DMA2_Stream1_IRQn);
}

//-----------------------------------------------------------------------------
// 	void CMSIS_TIM1_Init(void)
//-----------------------------------------------------------------------------

void CMSIS_TIM1_Init(void){
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //Enable GPIOA

	TIM1->PSC = 0;
	TIM1->ARR = 1;

	TIM1->CR1 &= (uint32_t) (~TIM_CR1_ARPE); //Auto-reload preload enable

	TIM1->CCMR1 |= (uint32_t) TIM_CCMR1_CC1S_0; //CC1 channel is conf as input
	TIM1->CCMR1 &= (uint32_t) (~(TIM_CCMR1_IC1F | TIM_CCMR1_IC1PSC));

	TIM1->CCER &= (uint32_t) ~TIM_CCER_CC1P; //Rising edge
	//TIM1->CCER |= (uint32_t) TIM_CCER_CC1P; //Falling edge
	//TIM8->CCER |= TIM_CCER_CC1NP; //Both edge
	TIM1->CCER |= (uint32_t) TIM_CCER_CC1E;

	TIM1->DIER |= (uint32_t) TIM_DIER_CC1DE; //Allow interruption by DMA

	for (uint16_t i = 0; i < 0xffff; i++); //delay for avoiding fatal error

	NVIC_EnableIRQ(TIM1_CC_IRQn); // TIM1 global interrupt enable
}

//-----------------------------------------------------------------------------
// 	void TIM8_Init(void)
//	@brief This function init TIM8 in INput Capture/Compare mode CH1 and allow
//		   interruption by DMA
//-----------------------------------------------------------------------------

void CMSIS_TIM8_Init(void){
	RCC->APB2ENR |= RCC_APB2ENR_TIM8EN; // Enable TIM8
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //Enable GPIOA

	TIM8->PSC = 0;
	TIM8->ARR = 420;

	TIM8->SMCR |= TIM_SMCR_ECE; // External clock mode 2 enable
	TIM8->SMCR |= TIM_SMCR_ETP; //ETR is inverted

	TIM8->DIER |= TIM_DIER_UDE; // Update DMA request enable
}

//	Using Input Capture/Compare mode
/*
void CMSIS_TIM8_Init(void){
	RCC->APB2ENR |= RCC_APB2ENR_TIM8EN; // Enable TIM8
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //Enable GPIOA
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; //Enable GPIOC

	TIM8->PSC = 0;
	TIM8->ARR = 1;

	TIM8->CR1 &= (~TIM_CR1_ARPE); //Auto-reload preload enable

	TIM8->CCMR1 |= TIM_CCMR1_CC1S_0; //CC1 channel is conf as input
	TIM8->CCMR1 &= (~(TIM_CCMR1_IC1F | TIM_CCMR1_IC1PSC));

	TIM8->CCER |= TIM_CCER_CC1P; //Falling edge
	//TIM8->CCER |= TIM_CCER_CC1NP; //Both edge
	TIM8->CCER |= TIM_CCER_CC1E;
	TIM8->DIER |= TIM_DIER_CC1DE; //Allow interruption by DMA

	for (uint16_t i = 0; i < 0xffff; i++); //delay for avoiding fatal error

	NVIC_EnableIRQ(TIM8_CC_IRQn); // TIM8 global interrupt enable
}*/

//-----------------------------------------------------------------------------
// 	void CMSIS_TIM2_Init(void)
//-----------------------------------------------------------------------------

void CMSIS_TIM2_Init(void){
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //Enable GPIOA
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable TIM2
	//RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;// Enable alternative functions

	//Config friq
	TIM2->PSC = (uint32_t) 1; //freq 1kHz
	TIM2->ARR = (uint32_t) 0xffff;

	//CR1
	TIM2->CR1 &= ~TIM_CR1_DIR; //Counter used as upcounter
	TIM2->CR1 &= ~TIM_CR1_ARPE; //Auto-reload preload enable
	TIM2->CR1 &= ~TIM_CR1_CKD; //Clock division 1

	//DIER
	TIM2->DIER |= TIM_DIER_CC2IE; //Capture/Compare 3/4 enable
	TIM2->DIER |= TIM_DIER_CC4IE;

	//CCMR2
	TIM2->CCMR1 &= ~TIM_CCMR1_IC2F;
	TIM2->CCMR2 &= ~TIM_CCMR2_IC4PSC; //don't filter
	TIM2->CCMR1 &= ~TIM_CCMR1_IC2PSC;
	TIM2->CCMR2 &= ~TIM_CCMR2_IC4F; //don't use prescaler

	TIM2->CCMR1 &= ~TIM_CCMR1_CC2S;
	TIM2->CCMR2 &= ~TIM_CCMR2_CC4S;
	TIM2->CCMR1 |= TIM_CCMR1_CC2S_0;//: CC2 channel is configured as input, IC2 is mapped on TI2
	TIM2->CCMR2 |= TIM_CCMR2_CC4S_0;//: CC4 channel is configured as input, IC4 is mapped on TI4

	//CCER
	TIM2->CCER &= ~TIM_CCER_CC2P; //Rising mode
	TIM2->CCER |= TIM_CCER_CC4P; //Falling mode
	TIM2->CCER |= TIM_CCER_CC2E | TIM_CCER_CC4E; //Capture enable

	TIM2->CR1 |= TIM_CR1_CEN; //Counter enable

	NVIC_EnableIRQ(TIM2_IRQn); // TIM2 global interrupt enable
}

//-----------------------------------------------------------------------------
// 	void CMSIS_EXTI_Init(void)
//	@brief This function init EXTI for handle SYNC signal XY2-100 protocol
//-----------------------------------------------------------------------------

void CMSIS_EXTI_Init(void){
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // Enable alternative functions

	// PA2 configure - SYNC
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PA; //EXTI2 on port A
	EXTI->IMR |= EXTI_IMR_MR2;  //Enable interrupts from EXTI2
	EXTI->RTSR |= EXTI_RTSR_TR2;

	NVIC_EnableIRQ(EXTI2_IRQn);
}

void find_offset(uint16_t* buf_GPIO){
	offset_idx = 0;
	while ((buf_GPIO[offset_idx] & 0x4) != 0) {
		offset_idx++;
	}
	if (DATA_XY2_LEN - offset_idx != 0) {
		offset_idx = DATA_XY2_LEN - offset_idx;
	} else {
		offset_idx = 0;
	}
}

void data_Processing(uint16_t* buf_GPIO, struct Data_XY2_100* buf_data, uint16_t* buf_sync, uint16_t buf_size){
	// We take into account the data offset, so the value of the initial bit is "1"
	uint8_t current_bit = 0x0;
	uint16_t current_frame = 0x0;

	for (uint16_t i = offset_idx - 1; i < buf_size - offset_idx - 1; ++i){
		current_bit++;
		if (current_bit > 2 && current_bit < 19) {
			/*
			 * recording each bit taking into account its location
			 */
#if 0
			buf_data[current_frame].x |= ((buf_GPIO[i] >> DATA_X_OFFSET) & 0x1) << (18 - current_bit);
			buf_data[current_frame].y |= ((buf_GPIO[i] >> DATA_Y_OFFSET) & 0x1) << (18 - current_bit);
			buf_data[current_frame].z |= ((buf_GPIO[i] >> DATA_Z_OFFSET) & 0x1) << (18 - current_bit);
#endif
#if 1
			data_buf_x[current_frame] |= ((buf_GPIO[i] >> DATA_X_OFFSET) & 0x1) << (18 - current_bit);
			data_buf_y[current_frame] |= ((buf_GPIO[i] >> DATA_Y_OFFSET) & 0x1) << (18 - current_bit);
			data_buf_z[current_frame] |= ((buf_GPIO[i] >> DATA_Z_OFFSET) & 0x1) << (18 - current_bit);
#endif
		} else if (current_bit == 19){
			current_bit = 0xff;
			/*
			 * check parity even
			 */
#if 1
			if (!(calc_PE(data_buf_x[current_frame], ((buf_GPIO[i] >> DATA_X_OFFSET) & 0x1), DATA_XY2_LEN)
					&& calc_PE(data_buf_y[current_frame], ((buf_GPIO[i] >> DATA_Y_OFFSET) & 0x1), DATA_XY2_LEN)
					&& calc_PE(data_buf_z[current_frame], ((buf_GPIO[i] >> DATA_Z_OFFSET) & 0x1), DATA_XY2_LEN))) {
				fault_frames[fault_frames_idx] = current_frame;
				fault_frames_idx++;
			}
#endif
#if 0
			if (!(calc_PE(buf_data[current_frame].x, ((buf_GPIO[i] >> DATA_X_OFFSET) & 0x1), DATA_XY2_LEN)
					&& calc_PE(buf_data[current_frame].y, ((buf_GPIO[i] >> DATA_Y_OFFSET) & 0x1), DATA_XY2_LEN)
					&& calc_PE(buf_data[current_frame].z, ((buf_GPIO[i] >> DATA_Z_OFFSET) & 0x1), DATA_XY2_LEN))) {
				fault_frames[fault_frames_idx] = current_frame;
				fault_frames_idx++;
			}
#endif
			current_frame++;
		}
	}
}

//-----------------------------------------------------------------------------
// 	uint8_t calc_PE(uint16_t data, uint8_t PE, uint8_t len)
//-----------------------------------------------------------------------------

uint8_t calc_PE(uint16_t data, uint8_t PE, uint8_t len){
	uint8_t sum = 0;

	for (uint8_t i = 0; i < len; i++){
		if ((data & 0x1) == 1){
			sum++;
		}
		data = data >> 0x1;
	}

	sum++; // We take into account the unit in the first three bits
	uint8_t temp = (uint8_t)sum & 0x1;
	return PE == temp; // if PE and junior bit of sum are equal
}

//-----------------------------------------------------------------------------
// 	void EXTI2_IRQHandler(void)
//	@brief This function enable DMA2 store GPIOx values and prohibit
//		   to use EXTI2
//-----------------------------------------------------------------------------

void EXTI2_IRQHandler(void){
	EXTI->IMR &= ~EXTI_IMR_MR2; //Disable interrupts from EXTI2
	TIM8->CR1 |= TIM_CR1_CEN;
	//TIM8->CCER |= (uint32_t)TIM_CCER_CC1E;
	EXTI->PR |= EXTI_PR_PR2;
}

//-----------------------------------------------------------------------------
// 	void EXTI1_IRQHandler(void)
//-----------------------------------------------------------------------------
void EXTI1_IRQHandler(void){

}
