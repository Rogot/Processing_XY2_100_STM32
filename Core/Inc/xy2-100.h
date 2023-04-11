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

#define GPIOx_BUF_SIZE				40080
#define GPIOx_BUF_HALF_SIZE			GPIOx_BUF_SIZE / 2
#define DATA_BUF_SIZE				GPIOx_BUF_SIZE / 20 / 2
#define DATA_BUF_HALF_SIZE			DATA_BUF_SIZE / 2
#define DATA_XY2_LEN				20
#define DATA_XY2_USB_LEN			8

#define SYNC_OFFSET					2
#define DATA_X_OFFSET				10
#define DATA_Y_OFFSET				9
#define DATA_Z_OFFSET				6

#define CENTRAL_COORFINATE_X		32767
#define CENTRAL_COORDINATE_Y		32768

#define DC_BUFF_SIZE	2000	/* duty cycle buffer size */
#define DC_BUFF_HALF_SIZE	DC_BUFF_SIZE / 2	/* duty cycle half buffer size */
#define POWER_LASER_NORM_MAX	1
#define POWER_LASER_NORM_MIN	0

/* ~Defines~ */

extern frame_num, bit_num;
extern uint16_t fault_frames[256];
extern uint8_t fault_frames_idx;
extern uint8_t is_sync;
extern uint16_t GPIOx_offset_idx;
extern uint16_t data_offset_idx;
extern uint16_t sample_counter;
extern uint8_t flag;

extern uint32_t duty_cycle_buff[DC_BUFF_HALF_SIZE];
extern uint16_t pulseWidth;
extern uint16_t period;
extern uint16_t dutyCycle;
extern uint16_t CNTBegin;

//extern struct Data_XY2_100 data_buff[DATA_BUF_SIZE];
extern uint16_t sync_buff[GPIOx_BUF_SIZE];
extern uint16_t GPIOx_buff[GPIOx_BUF_SIZE];

typedef struct DATA_XYZ {
	uint16_t x;
	uint16_t y;
	//uint16_t z;
	uint16_t dutyCycle;
}t_DATA;

extern t_DATA data_buf[DATA_BUF_SIZE];
extern t_DATA data_buf_first[DATA_BUF_HALF_SIZE];
extern t_DATA data_buf_second[DATA_BUF_HALF_SIZE];


extern uint8_t proc_1_ready, proc_2_ready;
extern uint8_t proc_1_busy, proc_2_busy;
extern uint8_t trans_1_ready, trans_2_ready;
extern uint8_t trans_1_busy, trans_2_busy;
extern uint8_t overrun;


extern uint8_t FPBGP;
extern uint8_t FFP;
extern uint16_t last_bits;


extern uint32_t total_send;
extern uint32_t total_send_1;
extern uint32_t total_send_2;


extern uint32_t count;
extern uint32_t total_send;


/* Functions */
void CMSIS_GPIO_Init(void);
void CMSIS_EXTI_Init(void);
void CMSIS_TIM3_Init(void);
void CMSIS_TIM2_Init(void);
void CMSIS_TIM8_Init(void);
void CMSIS_DMA_Init(DMA_Stream_TypeDef* dma_stream);
void CMSIS_DMA_Config(DMA_Stream_TypeDef* dma_stream, uint32_t srcAdrr, uint32_t dstAdrr, uint16_t dataSize);

void find_offset(uint16_t* buf_GPIO);
//void data_processing(uint16_t* GPIO_buf, uint16_t GPIO_buf_size, uint16_t start_addr_gpio_buf, uint16_t start_addr_data_buf);
void data_processing_test(t_DATA* data_buf, uint16_t* GPIO_buf, uint16_t* iterrator, uint16_t GPIO_buf_size, uint16_t start_addr_gpio_buf);
uint8_t calc_PE(uint16_t data, uint8_t PE, uint8_t len);

/* ~Functions~ */

#endif
