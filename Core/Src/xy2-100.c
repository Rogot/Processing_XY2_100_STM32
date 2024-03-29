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
	/*	PA9 - X	*/
	GPIOA->MODER &= ~GPIO_MODER_MODE9; // Input mode
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9_1; //High speed

	/*	PA7 - Y	*/
	GPIOA->MODER &= ~GPIO_MODER_MODE7; // Input mode
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7_1; //High speed

	/*	PA6 - Z	*/
	GPIOA->MODER &= ~GPIO_MODER_MODE6; // Input mode
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6_1; //High speed

	/*	PC6 - CLCK*/
	GPIOC->MODER |= GPIO_MODER_MODE6_1; // Alternative mode
	GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6_1; //High speed
	GPIOC->AFR[0] |= GPIO_AFRL_AFRL6_0 | GPIO_AFRL_AFRL6_1; //Alternate function for PC6 - AF3 (TIM8..TIM11)

	/*	PA2 - SYNC	*/
	GPIOA->MODER &= ~GPIO_MODER_MODE2; // Input mode
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR2_1; //High speed

	/* ~Информационные пины~ */
//
//	// PA1 - TIM2 input channel 2
//	GPIOA->MODER &= ~GPIO_MODER_MODE1;
//	GPIOA->MODER |= GPIO_MODER_MODE1_1; // Alternate function mode
//	//GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR1_1; //High speed
//	//GPIOA->PUPDR |= GPIO_PUPDR_PUPD1_1; //Pull-down
//	GPIOA->AFR[0] |= GPIO_AFRL_AFRL1_0; //Alternate function for PA1 - AF1 (TIM1/TIM2)
//
//	// PA3 - TIM2 input channel 4
//	GPIOA->MODER &= ~GPIO_MODER_MODE3;
//	GPIOA->MODER |= GPIO_MODER_MODE3_1; // Alternate function mode
//	//GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR3_1; //High speed
//	//GPIOA->PUPDR |= GPIO_PUPDR_PUPD3_1; //Pull-down
//	GPIOA->AFR[0] |= GPIO_AFRL_AFRL3_0; //Alternate function for PA3 - AF1 (TIM1/TIM2)


	/* TEMP (проверка работы на осциллографе) - удалить */

	/* PA4 - шип 1 */
	GPIOA->MODER &= ~GPIO_MODER_MODE4;
	GPIOA->MODER |= GPIO_MODER_MODE4_0; //Output mode
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4_1;

	/* PA7 - шип 2*/
//	GPIOA->MODER &= ~GPIO_MODER_MODE7;
//	GPIOA->MODER |= GPIO_MODER_MODE7_0; //Output mode
//	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7_1;
	/* TEMP (проверка работы DMA) - удалить */

}

//-----------------------------------------------------------------------------
// 	void CMSIS_DMA_Init(DMA_Stream_TypeDef* dma_stream)
//	@brief This function init DMA2 for working with TIM8
//-----------------------------------------------------------------------------

void CMSIS_DMA_Init(DMA_Stream_TypeDef* dma_stream){

	/* Data recording XY2-100 */
	if (dma_stream == DMA2_Stream2) {

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
	/* ~Data recording XY2-100~ */

	/* Recording data about laser power */
	else if (dma_stream == DMA2_Stream1) {
		RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN; /* Enable DMA2 */

		dma_stream->CR |= DMA_SxCR_CHSEL_1 | DMA_SxCR_CHSEL_2; /* Channel 6 */
		dma_stream->CR &= ~DMA_SxCR_DIR; /* Peripheral-to-memory */
		dma_stream->CR |= DMA_SxCR_CIRC; //Circular mode enable
		dma_stream->CR |= DMA_SxCR_PL_1; //Priority level high
		dma_stream->CR |= DMA_SxCR_MSIZE_0; // 16 bit
		dma_stream->CR |= DMA_SxCR_PSIZE_0; // 16 bit
		dma_stream->CR &= ~DMA_SxCR_MINC; //Memory increment mode enable
		dma_stream->CR &= ~DMA_SxCR_PINC; //Peripheral increment mode disable
//		dma_stream->CR |= DMA_SxCR_TCIE; //Interrupt enable
//		dma_stream->CR |= DMA_SxCR_HTIE; //Interrupt half enable
	}
	else if (dma_stream == DMA2_Stream6) {
		RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN; /* Enable DMA2 */

		dma_stream->CR |= DMA_SxCR_CHSEL_1 | DMA_SxCR_CHSEL_2; /* Channel 6 */
		dma_stream->CR &= ~DMA_SxCR_DIR; /* Peripheral-to-memory */
		dma_stream->CR |= DMA_SxCR_CIRC; //Circular mode enable
		dma_stream->CR |= DMA_SxCR_PL_1; //Priority level high
		dma_stream->CR |= DMA_SxCR_PSIZE_0; // 16 bit
		dma_stream->CR |= DMA_SxCR_MSIZE_0; // 16 bit
		dma_stream->CR |= DMA_SxCR_MINC; //Memory increment mode enable
		dma_stream->CR &= ~DMA_SxCR_PINC; //Peripheral increment mode disable
//		dma_stream->CR |= DMA_SxCR_TCIE; //Interrupt enable
//		dma_stream->CR |= DMA_SxCR_HTIE; //Interrupt half enable
	}
	/* ~Recording data about laser power~ */
}

//-----------------------------------------------------------------------------
// 	void CMSIS_DMA_Config(DMA_Stream_TypeDef* dma_stream, uint32_t srcAdrr, uint32_t dstAdrr, uint16_t dataSize)
//	@brief This function configure DMA
//-----------------------------------------------------------------------------

void CMSIS_DMA_Config(DMA_Stream_TypeDef* dma_stream, uint32_t srcAdrr, uint32_t dstAdrr, uint16_t dataSize){
	dma_stream->PAR = (uint32_t) srcAdrr;

	dma_stream->M0AR = (uint32_t) dstAdrr;

	dma_stream->NDTR = dataSize; //Buffer size

	dma_stream->CR |= DMA_SxCR_EN; //Stream enable

	if (dma_stream == DMA2_Stream2) {
		NVIC_EnableIRQ(DMA2_Stream2_IRQn);
	}
}

//-----------------------------------------------------------------------------
// 	void CMSIS_TIM1_Init(void)
//-----------------------------------------------------------------------------

void CMSIS_TIM3_Init(void){
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //Enable GPIOA

	TIM3->SMCR &= ~ TIM_SMCR_SMS;

	TIM3->PSC = 0xff; //169 MHz / 65536 = 2563 Hz
	TIM3->ARR = 0x140A; // 2563 Hz / 2565 = 1 Hz

	TIM3->CR1 &= TIM_CR1_DIR; //upcounter mode
	TIM3->DIER |= TIM_DIER_UIE; //interrapt enable

	for (uint16_t i = 0; i < 0xffff; i++); //delay for avoiding fatal error

	NVIC_EnableIRQ (TIM3_IRQn);
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

	TIM8->CR1 &= (uint32_t) (~TIM_CR1_ARPE); //Auto-reload preload disable

	TIM8->CCMR1 |= (uint32_t) TIM_CCMR1_CC1S_0; //CC1 channel is conf as input
	TIM8->CCMR1 &= (uint32_t) (~(TIM_CCMR1_IC1F | TIM_CCMR1_IC1PSC));

	//TIM8->CCER &= (uint32_t) ~TIM_CCER_CC1P; //Rising edge
	TIM8->CCER |= (uint32_t) TIM_CCER_CC1P; //Falling edge
	//TIM8->CCER |= TIM_CCER_CC1NP; //Both edge
	TIM8->CCER |= (uint32_t) TIM_CCER_CC1E;
//	TIM8->DIER |= (uint32_t) TIM_DIER_CC1DE; //Allow interruption by DMA

	for (uint16_t i = 0; i < 0xffff; i++); //delay for avoiding fatal error

	NVIC_EnableIRQ(TIM8_CC_IRQn); // TIM8 global interrupt enable
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
	if (DATA_XY2_LEN - GPIOx_offset_idx > 1) {
		GPIOx_offset_idx = DATA_XY2_LEN - GPIOx_offset_idx - 1;
		data_offset_idx = 1;
	}
	else {
		GPIOx_offset_idx = 0;
		data_offset_idx = 0;
	}
}

void data_processing_test(t_DATA *data_buf, uint16_t *GPIO_buf, uint16_t* iterrator,
		uint16_t GPIO_buf_size, uint16_t start_addr_gpio_buf) {
	// We take into account the data offset, so the value of the initial bit is "1"
	int8_t current_bit;
	int8_t temp_i;
	uint16_t meas_cur_1, meas_cur_2, meas_prev_1 = 0;

	if (start_addr_gpio_buf == GPIOx_BUF_HALF_SIZE - GPIOx_offset_idx){
		sample_counter = DC_BUFF_HALF_SIZE;
	} else {
		sample_counter = 0;
	}

	*iterrator = 0;
	period = TIM1->CCR1;

	uint16_t x = 0, y = 0, z = 0;

	if (FPBGP) {
		current_bit = 15;
		uint16_t t = GPIOx_BUF_SIZE - GPIOx_offset_idx + 3;
		for (; t < GPIOx_BUF_SIZE; t++) {
			x |= ((GPIO_buf[t] >> DATA_X_OFFSET) & 0x1) << (current_bit);
			y |= ((GPIO_buf[t] >> DATA_Y_OFFSET) & 0x1) << (current_bit);
			z |= ((GPIO_buf[t] >> DATA_Z_OFFSET) & 0x1) << (current_bit);
			current_bit--;
		}

		if (3 - GPIOx_offset_idx < 0) {
			t = 0;
		} else {
			t = 3 - GPIOx_offset_idx;
		}

		for (; current_bit > -1; current_bit--) {
			x |= ((GPIO_buf[t] >> DATA_X_OFFSET) & 0x1) << (current_bit);
			y |= ((GPIO_buf[t] >> DATA_Y_OFFSET) & 0x1) << (current_bit);
			z |= ((GPIO_buf[t] >> DATA_Z_OFFSET) & 0x1) << (current_bit);
			t++;
		}

		if (flag) {
			if (FFP) {
				data_buf[*iterrator].x = x;
				data_buf[*iterrator].y = y;
				//data_buf[*iterrator].z = z;
				pulseWidth = duty_cycle_buff[sample_counter + (*iterrator)];
				if (period) {

					p_f = ((float) pulseWidth / (float) period);

					if (p_f < 1.0) {

						uint16_t p_16 = (p_f - POWER_LASER_NORM_MIN)
								/ (POWER_LASER_NORM_MAX - POWER_LASER_NORM_MIN)
								* 65535;

						data_buf[*iterrator].dutyCycle = p_16;
					}
				}
			} else {
				FFP = 0x1;
			}

			x = 0x0;
			y = 0x0;
			z = 0x0;

			if (*iterrator < DATA_BUF_SIZE) {
				*iterrator += 1;
//				sample_counter++;
			} else {
				*iterrator = 0x0;
//				sample_counter = 0x0;
			}
		}
	}

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

		if (flag) {
			if (x != CENTRAL_COORFINATE_X && y != CENTRAL_COORDINATE_Y ){
				TIM3->CR1 &= ~TIM_CR1_CEN;
			} else {
				TIM3->CR1 |= TIM_CR1_CEN;
			}
			pulseWidth = duty_cycle_buff[sample_counter + (*iterrator)];

			data_buf[*iterrator].x = x;
			data_buf[*iterrator].y = y;
			//data_buf[*iterrator].z = z;
			if (period) {

				p_f = ((float) pulseWidth / (float) period);

				if (p_f < 1.0) {

					uint16_t p_16 = (p_f - POWER_LASER_NORM_MIN)
							/ (POWER_LASER_NORM_MAX - POWER_LASER_NORM_MIN)
							* 65535;

					data_buf[*iterrator].dutyCycle = p_16;
				}
			}

			if (*iterrator < DATA_BUF_SIZE) {
				*iterrator += 1;
			} else {
				*iterrator = 0x0;
			}

//			if (!(calc_PE(x, ((GPIO_buf[temp_i] >> DATA_X_OFFSET) & 0x1), 16)
//					&& calc_PE(y, ((GPIO_buf[temp_i] >> DATA_Y_OFFSET) & 0x1),
//							16)
//			//&& calc_PE(z,
//			//((GPIO_buf[i + 16] >> DATA_Z_OFFSET) & 0x1), DATA_XY2_LEN)
//			)) {
//				fault_frames[fault_frames_idx] = *iterrator;
//				fault_frames_idx++;
//				if (fault_frames_idx > 256) {
//					fault_frames_idx = 0;
//				}
//			}

		} else if ((x != CENTRAL_COORFINATE_X && y != CENTRAL_COORDINATE_Y)
				&& (x != 0 && y != 0)) {
			flag = 0x1;
		}

		x = 0x0;
		y = 0x0;
		z = 0x0;

		i += DATA_XY2_LEN * 2; /* Missed several coordinate for optimization */
		sample_counter += 2;
	}
}

//-----------------------------------------------------------------------------
// 	uint8_t calc_PE(uint16_t data, uint8_t PE, uint8_t len)
//-----------------------------------------------------------------------------

uint8_t calc_PE(uint16_t data, uint8_t PE, uint8_t len){
	uint8_t sum = 0;

	for (uint8_t idx = 0; idx < len; idx++){
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
