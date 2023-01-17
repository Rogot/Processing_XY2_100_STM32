#ifndef XY2_100_H_H
#define XY2_100_H_H

#include "main.h"
#include <stm32f405xx.h>
#include <stdio.h>

/* Macros */
#define CLK_Read 					(GPIOB->IDR & GPIO_IDR_IDR_0)
#define X_Read 						((GPIOA->IDR & GPIO_IDR_IDR_10))
#define Y_Read 						(GPIOA->IDR & GPIO_IDR_IDR_9)
#define Z_Read 						(GPIOA->IDR & GPIO_IDR_IDR_6)
/* ~Macros~ */

/* Defines */
#define TIM2_Address				((uint32_t)0x40000000)
#define TIM2_CCR1_Address			((uint32_t)TIM2_Address + 0x34)
#define TIM2_CCR2_Address			((uint32_t)TIM2_Address + 0x38)

#define GPIOx_BUF_SIZE				20000
#define DATA_BUF_SIZE				1000
#define DATA_XY2_LEN				20
#define DATA_XY2_USB_LEN			8

#define SYNC_OFFSET					2
#define DATA_X_OFFSET				10
#define DATA_Y_OFFSET				9
#define DATA_Z_OFFSET				6
/* ~Defines~ */

#define CENTRAL_COORFINATE_X		32767
#define CENTRAL_COORDINATE_Y		32768

//struct Data_XY2_100{
//	uint16_t x;
//	uint16_t y;
//	uint16_t z;
//};

extern frame_num, bit_num;
extern uint32_t fault_frames[256];
extern uint32_t fault_frames_idx;
extern uint8_t is_sync;
extern uint16_t offset_idx;

//extern struct Data_XY2_100 data_buff[DATA_BUF_SIZE];
extern uint16_t sync_buff[GPIOx_BUF_SIZE];
extern uint16_t GPIOx_buff[GPIOx_BUF_SIZE];

extern uint16_t data_buf_x[DATA_BUF_SIZE];
extern uint16_t data_buf_y[DATA_BUF_SIZE];
extern uint16_t data_buf_z[DATA_BUF_SIZE];


/* Functions */
void CMSIS_GPIO_Init(void);
void CMSIS_EXTI_Init(void);
void CMSIS_TIM1_Init(void);
void CMSIS_TIM2_Init(void);
void CMSIS_TIM8_Init(void);
void CMSIS_DMA_Init(DMA_Stream_TypeDef* dma_stream);
void CMSIS_DMA_Config(DMA_Stream_TypeDef* dma_stream, uint32_t srcAdrr, uint32_t dstAdrr, uint16_t dataSize);

void find_offset(uint16_t* buf_GPIO);
void data_processing(uint16_t* buf_GPIO, uint16_t* buf_sync, uint16_t buf_size);
//void data_processing(uint16_t* buf_GPIO, struct Data_XY2_100* buf_data, uint16_t* buf_sync, uint16_t buf_size);
uint8_t calc_PE(uint16_t data, uint8_t PE, uint8_t len);

/* ~Functions~ */

#endif
