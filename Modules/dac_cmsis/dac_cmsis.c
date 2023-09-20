#include "dac_cmsis.h"

void CMSIS_DAC_init(t_dac* dac) {
	
	TIM_init(dac->tim, dac->tim_presc, dac->tim_arr);
	
	RCC->APB1ENR |= RCC_APB1ENR_DACEN; /* Enable DAC */

	dac->dac_type->CR |= DAC_CR_TEN1; /* Input data on by event */

	dac->dac_type->CR &= ~DAC_CR_TSEL1; /* Software control bit */ 
	
	//dac->dac_type->CR |= DAC_CR_BOFF1; /* output budder off */
	
	
	
	dac->dac_type->CR |= DAC_CR_EN1; 
}

void TIM_init(TIM_TypeDef* tim, uint16_t psc, uint16_t arr) {
	 RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;             // TIM6 enable

   tim->PSC = psc;                             
   tim->ARR = arr;                                

   tim->DIER |= TIM_DIER_UIE;             
   tim->DIER |= TIM_DIER_UDE;             
                
   tim->CR2 |= TIM_CR2_MMS_1;     
   tim->CR1 |= TIM_CR1_CEN;       
}
