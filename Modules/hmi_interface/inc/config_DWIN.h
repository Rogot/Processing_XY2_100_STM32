#ifndef CONFIG_DWIN_H
#define CONFIG_DWIN_H

#include <stm32f405xx.h>

#define DWIN_SERIAL_PORT_ENABLE			( 1 )

#define DAC_ENABLE									( 0 )

#define DMA_ENABLE									( 0 )

#define DWIN_USART_ENABLE						( 0 )

#define DATA_USART2_TX							( 0 )

#define MODBUS_ENABLE								( 0 ) 

#define STEP_ENGINE_ENABLE		 			( 0 )

#define STEP_ENGINE_TEST_ENABLE			( 0 )

#define PEREPH_ENABLE								( 0 )

#define HAL_ADC_MODULE_ENABLED			( 0 )

typedef uint8_t BOOL;

typedef unsigned char UCHAR;
typedef char CHAR;

typedef uint16_t USHORT;
typedef int16_t SHORT;

typedef uint32_t ULONG;
typedef int32_t LONG;

#endif //!CONFIG_DWIN_H