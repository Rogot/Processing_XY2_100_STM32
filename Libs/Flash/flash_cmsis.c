#include "flash_cmsis.h"

void CMSIS_flash_allow_access(void) {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	
	FLASH->KEYR = FLASH_KEY1;
	FLASH->KEYR = FLASH_KEY2;
}

/*
* @bref: page_address - any address belong to erase page/sector.
*/
void CMSIS_internal_flash_erase(uint16_t sec_num) {
	while (FLASH->SR & FLASH_SR_BSY);
	if (FLASH->SR & FLASH_SR_EOP) {
		FLASH->SR = FLASH_SR_EOP;
	}
	
	FLASH->CR |= FLASH_CR_SER; 	/* Sector erase activated */
	FLASH->CR |= sec_num; 			/* Select a sector to erase */
	FLASH->CR |= FLASH_CR_STRT;
	while (FLASH->SR & FLASH_SR_BSY);
}

void CMSIS_internal_flash_bank_erase(void) {
	while (FLASH->SR & FLASH_SR_BSY);
	if (FLASH->SR & FLASH_SR_EOP) {
		FLASH->SR = FLASH_SR_EOP;
	}
	
	FLASH->CR |= FLASH_CR_MER; /* Mass erase activated */
	
	FLASH->CR |= FLASH_CR_STRT;
	while (FLASH->SR & FLASH_SR_BSY);
}

/*
* @bref: data - pointer to the data being recorded;
				 address - address in the FLASH;
				 len - count of writing data.
*/
void CMSIS_internal_flash_write(uint8_t* data, uint32_t address, uint16_t len) {
	uint16_t i;
	
	while (FLASH->SR & FLASH_SR_BSY);
	if (FLASH->SR & FLASH_SR_EOP) {
		FLASH->SR = FLASH_SR_EOP;
	}
	
	FLASH->CR |= FLASH_CR_PG; /* Sector programm activated */
	
	for (i = 0; i < len; i++) {
		*(volatile uint8_t*) (address + i) = data[i];
		//while (!(FLASH->SR & FLASH_SR_EOP));
		FLASH->SR = FLASH_SR_EOP;
	}
	
	FLASH->CR &= ~FLASH_CR_PG;
}

/*
* @bref: address - address in the FLASH.
*/
uint32_t CMSIS_internal_flash_read(uint32_t address) {
	return (*(__IO uint32_t*)address);
}