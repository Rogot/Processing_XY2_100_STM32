#ifndef DAC_CMSIS_H
#define DAC_CMSIS_H

#include <stm32f405xx.h>
#include "stm32f4xx_hal.h"
#include "config_DWIN.h"

typedef struct DAC_CMSIS {
	TIM_TypeDef* tim;
	DAC_TypeDef* dac_type;
	uint16_t tim_presc;
	uint16_t tim_arr;
} t_dac;

void CMSIS_DAC_init(t_dac* dac);

void TIM_init(TIM_TypeDef* tim, uint16_t psc, uint16_t arr);


#endif //!DAC_CMSIS_H