#include "xy2-100.h"

//-----------------------------------------------------------------------------
// 	void CMSIS_GPIO_Init(void)
//	@brief This function init GPIOx
//-----------------------------------------------------------------------------

void CMSIS_GPIO_Init(void){
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; //Enable GPIOB
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //Enable GPIOA
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; //Enable GPIOC

	/* Информационные пины */
	/*	PA10 - X	*/
	GPIOA->MODER &= ~GPIO_MODER_MODE10; // Input mode
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR10_1; //High speed

	/*	PA9 - Y	*/
	GPIOA->MODER &= ~GPIO_MODER_MODE9; // Input mode
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9_1; //High speed

	/*	PA6 - Z	*/
	GPIOA->MODER &= ~GPIO_MODER_MODE6; // Input mode
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6_1; //High speed

//	/*	PA0 - CLCK*/
//	GPIOA->MODER |= GPIO_MODER_MODE0_1; // Alternative mode
//	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR0_1; //High speed
//	GPIOA->AFR[0] |= GPIO_AFRL_AFRL0_0 | GPIO_AFRL_AFRL0_1; //Alternate function for PA0 - AF3 (TIM8..TIM11)

	/*	PC6 - CLCK*/
	GPIOC->MODER |= GPIO_MODER_MODE6_1; // Alternative mode
	GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6_1; //High speed
	GPIOC->AFR[0] |= GPIO_AFRL_AFRL6_0 | GPIO_AFRL_AFRL6_1; //Alternate function for PC6 - AF3 (TIM8..TIM11)

	/*	PA2 - SYNC	*/
	GPIOA->MODER &= ~GPIO_MODER_MODE2; // Input mode
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR2_1; //High speed

	/* ~Информационные пины~ */

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


	/* TEMP (проверка работы DMA) - удалить */

	/* PA4 - время работы DMA */
	GPIOA->MODER &= ~GPIO_MODER_MODE4;
	GPIOA->MODER |= GPIO_MODER_MODE4_0; //Output mode
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4_1;

	/* TEMP (проверка работы DMA) - удалить */

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
	dma_stream->CR |= DMA_SxCR_HTIE; //Interrupt half enable
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

	NVIC_EnableIRQ(DMA2_Stream2_IRQn);
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
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; //Enable GPIOC

	TIM8->PSC = 0;
	TIM8->ARR = 1;

	TIM8->CR1 &= (uint32_t) (~TIM_CR1_ARPE); //Auto-reload preload enable

	TIM8->CCMR1 |= (uint32_t) TIM_CCMR1_CC1S_0; //CC1 channel is conf as input
	TIM8->CCMR1 &= (uint32_t) (~(TIM_CCMR1_IC1F | TIM_CCMR1_IC1PSC));

	TIM8->CCER &= (uint32_t) ~TIM_CCER_CC1P; //Rising edge
	//TIM8->CCER |= TIM_CCER_CC1NP; //Both edge
	TIM8->CCER |= (uint32_t) TIM_CCER_CC1E;
	TIM8->DIER |= (uint32_t) TIM_DIER_CC1DE; //Allow interruption by DMA

	for (uint16_t i = 0; i < 0xffff; i++); //delay for avoiding fatal error

	NVIC_EnableIRQ(TIM8_CC_IRQn); // TIM8 global interrupt enable
}

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
	GPIOx_offset_idx = 0;
	while ((buf_GPIO[GPIOx_offset_idx] & 0x4) != 0) {
		GPIOx_offset_idx++;
	}
	if (DATA_XY2_LEN - GPIOx_offset_idx != 0) {
		GPIOx_offset_idx = DATA_XY2_LEN - GPIOx_offset_idx - 1;
		data_offset_idx = 1;
	}
	else {
		GPIOx_offset_idx = 0;
		data_offset_idx = 0;
	}
}

void data_processing_test(uint16_t* GPIO_buf, uint16_t GPIO_buf_size, uint16_t start_addr_gpio_buf, uint16_t start_addr_data_buf){
	// We take into account the data offset, so the value of the initial bit is "1"
	//GPIOA->BSRR |= GPIO_BSRR_BS4;
	uint16_t prev_X = 0, prev_Y = 0;
	int8_t current_bit;
	//uint16_t current_frame = start_addr_data_buf;

	uint16_t x = 0, y = 0, z = 0;

	/*
	 * Внешний цикл - перемещение по фреймам, вложенный - обработка фрейма
	 * */

	for (uint16_t i = start_addr_gpio_buf + 3;
			i < GPIO_buf_size - GPIOx_offset_idx; i += DATA_XY2_LEN) {
		current_bit = 15;
		for (uint16_t j = i; j < i + 16; ++j) {
			x |= ((GPIO_buf[j] >> DATA_X_OFFSET) & 0x1) << (current_bit);
			y |= ((GPIO_buf[j] >> DATA_Y_OFFSET) & 0x1) << (current_bit);
			z |= ((GPIO_buf[j] >> DATA_Z_OFFSET) & 0x1) << (current_bit);
			current_bit--;
		}

		//if (flag && !sample_finished) {
		if (flag) {

			//if ((x < 49000 && y < 49000) && (x > 15000 && y > 15000)) {
				data_buf_x[sample_counter] = x;
				data_buf_y[sample_counter] = y;
				data_buf_z[sample_counter] = z;
			//}

			if (sample_counter < DATA_BUF_SIZE - 1) {
				sample_counter++;
			} else {
				//flag = 0x0;
				sample_finished = 0x1;
				sample_counter = 0x0;
			}
		} else if ((x != CENTRAL_COORFINATE_X && y != CENTRAL_COORDINATE_Y)
				&& (x != 0 && y != 0)) {
			flag = 0x1;
		}

//		if (prev_X == x && prev_Y == y && (x != CENTRAL_COORFINATE_X && y != CENTRAL_COORDINATE_Y)
//				&& (x != 0 && y != 0)){
//			transmission_end = 0x1;
//			transmission_end = 0x1;
//		}
//
//		prev_X = x;
//		prev_Y = y;

		x = 0x0;
		y = 0x0;
		z = 0x0;

		i += DATA_XY2_LEN; /* Missed several coordinate for optimization */
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
