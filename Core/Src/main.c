/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <usbd_cdc_if.h>
#include "xy2-100.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ITM_Port8(n)    (*((volatile unsigned char *)(0xE0000000+4*n)))
#define ITM_Port16(n)   (*((volatile unsigned short*)(0xE0000000+4*n)))
#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))

#define DEMCR           (*((volatile unsigned long *)(0xE000EDFC)))
#define TRCENA          0x01000000

#define SEND_ARR_SIZE	3
#define SEND_BYTE_SIZE 	SEND_ARR_SIZE * 2 * DATA_BUF_HALF_SIZE

uint32_t count = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* XY2_100 data processing variables */
uint32_t total_send = 0;
uint32_t total_send_1 = 0;
uint32_t total_send_2 = 0;

uint16_t sample_counter = 0x0;
uint8_t flag = 0x0, sample_finished = 0x0, transmission_end = 0x0;
uint8_t FPBGP = 0; /* First part buffer GPIO processed */
uint8_t FFP = 0; /* First frame processed */
uint16_t last_bits = 0;


uint8_t proc_1_ready = 0x0, proc_2_ready = 0x0;
uint8_t proc_1_busy = 0x0, proc_2_busy = 0x0;
uint8_t trans_1_ready = 0x0, trans_2_ready = 0x0;
uint8_t trans_1_busy = 0x0, trans_2_busy = 0x0;


uint8_t first_ready = 0x0, second_ready = 0x0;
uint8_t first_busy  = 0x0, second_busy = 0x1;
uint8_t overrun = 0x0;
t_DATA data_buf_first[DATA_BUF_HALF_SIZE] = {0};
t_DATA data_buf_second[DATA_BUF_HALF_SIZE] = {0};
uint16_t sample_iterrator_1 = 0, sample_iterrator_2 = 0;

uint16_t GPIOx_buff[GPIOx_BUF_SIZE] = {0};

uint16_t GPIOx_offset_idx = 0;
uint16_t data_offset_idx = 0;
uint8_t COF = 0x0; //Check offset flag

/* ~XY2_100 data processing variables~ */

/* Laser Power data processing variables */
uint32_t period;
uint32_t pulseWidth;

uint16_t duty_cycle_buff[DC_BUFF_SIZE] = {0}; /*borehole*/
uint16_t duty_cycle_period = 1;
/* ~Laser Power data processing variables~ */

uint16_t fault_frames[256] = {0};
uint8_t fault_frames_idx = 0x0;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	CMSIS_GPIO_Init();
	CMSIS_EXTI_Init();

	/* Config DMA for XY2-100 data */
	CMSIS_DMA_Init(DMA2_Stream2);
	CMSIS_DMA_Config(DMA2_Stream2, &GPIOA->IDR, (uint32_t) GPIOx_buff,
	GPIOx_BUF_SIZE);
	/* Config DMA for laser power data */
	CMSIS_DMA_Init(DMA2_Stream1);
	CMSIS_DMA_Config(DMA2_Stream1, &TIM1->CCR1, (uint32_t) &period,
	1);

	CMSIS_DMA_Init(DMA2_Stream6);
	CMSIS_DMA_Config(DMA2_Stream6, &TIM1->CCR2, duty_cycle_buff,
	DC_BUFF_SIZE);

	CMSIS_TIM8_Init();
	CMSIS_TIM3_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

	uint8_t status;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		if (COF) {
			if (proc_1_ready) {
				overrun = 0x0;
				proc_1_busy = 0x1;
				FPBGP = 0x1;

				data_processing_test(data_buf_first, GPIOx_buff,
						&sample_iterrator_1,
						GPIOx_BUF_HALF_SIZE, DATA_XY2_LEN - GPIOx_offset_idx);

				FPBGP = 0x0;

				proc_1_busy = 0x0;
				proc_1_ready = 0x0;
				trans_1_ready = 0x1;
			}

			if (proc_2_ready) {
				overrun = 0x0;
				proc_2_busy = 0x1;

				data_processing_test(data_buf_second, GPIOx_buff,
						&sample_iterrator_2,
						GPIOx_BUF_SIZE,
						GPIOx_BUF_HALF_SIZE - GPIOx_offset_idx);

				proc_2_busy = 0x0;
				proc_2_ready = 0x0;
				trans_2_ready = 0x1;
			}

			if (trans_1_ready && !trans_2_busy && flag) {
				trans_1_ready = 0x0;
				trans_1_busy = 0x1;
				status = CDC_Transmit_FS(data_buf_first,
				SEND_BYTE_SIZE);
				total_send_1 += SEND_BYTE_SIZE;
			}

			if (trans_2_ready && !trans_1_busy && flag) {
				trans_2_ready = 0x0;
				trans_2_busy = 0x1;
				status = CDC_Transmit_FS(data_buf_second,
				SEND_BYTE_SIZE);
				total_send_2 += SEND_BYTE_SIZE;
			}
		}
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 30000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

	TIM1->CCER |= (uint32_t) TIM_CCER_CC1E | (uint32_t) TIM_CCER_CC2E
			| (uint32_t) TIM_CCER_CC3E;

	TIM1->DIER |= TIM_DIER_CC1DE | TIM_DIER_CC2DE | TIM_DIER_CC3DE;
	TIM8->DIER |= (uint32_t) TIM_DIER_CC1DE; //Allow interruption by DMA
	TIM1->CR1 |= TIM_CR1_CEN;

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

void TIM3_IRQHandler(void) {
	flag = 0x0;
	TIM3->CR1 &= ~TIM_CR1_CEN;
	TIM3->SR &= ~TIM_SR_UIF;
}

void DMA2_Stream2_IRQHandler(void){

	if (!COF){
		find_offset(GPIOx_buff);
		COF = 0x1;
	}

	if ((DMA2->LISR & DMA_LISR_HTIF2) && !(DMA2->LISR & DMA_LISR_TCIF2)){
		DMA2->LIFCR |= DMA_LIFCR_CHTIF2;

		proc_1_ready = 0x1;
		if (proc_2_busy) {
			overrun = 0x1;
			proc_1_ready = 0x0;
		}
	}

	if (DMA2->LISR & DMA_LISR_TCIF2){
		DMA2->LIFCR |= DMA_LIFCR_CTCIF2;
		DMA2->LIFCR |= DMA_LIFCR_CHTIF2;

		proc_2_ready = 0x1;
		if (proc_1_busy){
			overrun = 0x1;
			proc_2_ready = 0x0;
		}
	}
}


void DMA2_Stream6_IRQHandler(void){
	TIM1->CNT = 0;
	DMA2->HIFCR |= DMA_HIFCR_CTCIF6;
	DMA2->HIFCR |= DMA_HIFCR_CHTIF6;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
