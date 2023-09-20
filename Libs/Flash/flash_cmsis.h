#ifndef FLASH_CMSIS_H
#define FLASH_CMSIS_H

#include <stm32f405xx.h>

#define FLASH_ENABLE		 ( 1 )

#define FLASH_KEY1			 0x45670123
#define FLASH_KEY2			 0xCDEF89AB

#define FLASH_SNB_SEC_0			 0x00
#define FLASH_SNB_SEC_1			 FLASH_CR_SNB_0
#define FLASH_SNB_SEC_2			 FLASH_CR_SNB_1
#define FLASH_SNB_SEC_3			 FLASH_CR_SNB_1 | FLASH_CR_SNB_0
#define FLASH_SNB_SEC_4			 FLASH_CR_SNB_2	
#define FLASH_SNB_SEC_5			 FLASH_CR_SNB_2 | FLASH_CR_SNB_0
#define FLASH_SNB_SEC_6 		 FLASH_CR_SNB_2 | FLASH_CR_SNB_1
#define FLASH_SNB_SEC_7			 FLASH_CR_SNB_2 | FLASH_CR_SNB_1 | FLASH_CR_SNB_0 
#define FLASH_SNB_SEC_8			 FLASH_CR_SNB_3 
#define FLASH_SNB_SEC_9			 FLASH_CR_SNB_3 | FLASH_CR_SNB_0
#define FLASH_SNB_SEC_10		 FLASH_CR_SNB_3 | FLASH_CR_SNB_1
#define FLASH_SNB_SEC_11		 FLASH_CR_SNB_3 | FLASH_CR_SNB_1 | FLASH_CR_SNB_0


void CMSIS_flash_allow_access(void); 

void CMSIS_internal_flash_erase(uint16_t sec_num);

void CMSIS_internal_flash_bank_erase(void);

void CMSIS_internal_flash_write(uint8_t* data, 
																uint32_t address, 
																uint16_t len);

uint32_t CMSIS_internal_flash_read(uint32_t address);


#endif //!FLASH_CMSIS_H