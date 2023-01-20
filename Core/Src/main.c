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
#include "xy2-100.h"
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

#define DC_BUFF_SIZE	100	/* duty cycle buffer size */
#define SEND_ARR_SIZE	3
#define SEND_BYTE_SIZE 	SEND_ARR_SIZE * 2


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

uint16_t sync_buff[GPIOx_BUF_SIZE] = {0};
//struct Data_XY2_100 data_buff[DATA_BUF_SIZE] = {0};

/*******TEST**********/
uint16_t data_buf_x[DATA_BUF_SIZE] = {0};
uint16_t data_buf_y[DATA_BUF_SIZE] = {0};
uint16_t data_buf_z[DATA_BUF_SIZE] = {0};
/*extra arrays for transmission data by 1 byte*/
uint16_t data_buf_tran[SEND_ARR_SIZE] = {0};
static int idx_frame = 0;
/******~TEST~*********/

uint16_t buff_impuls[2] = {0};
uint16_t GPIOx_buff[GPIOx_BUF_SIZE] = {0};

uint16_t offset_idx = 0;
uint8_t COF = 0x0; //Check offset flag

//uint16_t dma_buff[DMA_BUFF_SIZE];

/*	Timer	*/
uint16_t period;
uint16_t CNTCurrent;
uint16_t CNTBegin = 0;
uint16_t pulseWidth;
float dutyCycle;

float duty_cycle_buff[DC_BUFF_SIZE] = {0};
uint16_t duty_cycle_idx = 0;
/*	~Timer	*/

uint32_t fault_frames[256] = {0};
uint32_t fault_frames_idx = 0x0;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  CMSIS_GPIO_Init();
  CMSIS_EXTI_Init();
  //TIM2_Init();
  CMSIS_DMA_Init(DMA2_Stream1);
  CMSIS_DMA_Config(DMA2_Stream1, &GPIOA->IDR, (uint32_t)GPIOx_buff, GPIOx_BUF_SIZE);
  CMSIS_TIM8_Init();

  //HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
  //HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);

  uint8_t testDataToSend[8];
  for (uint8_t i = 0; i < 8; i++){
	  testDataToSend[i] = i;
  }
  testDataToSend[7] = '\n';

  uint16_t testDataToSending[SEND_ARR_SIZE];
  for (uint8_t i = 0; i < SEND_ARR_SIZE; i++){
	  testDataToSending[i] = i + 1;
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	//HAL_Delay(2000);
	//CDC_Transmit_FS(testDataToSend, DATA_XY2_USB_LEN);

	if (COF) {
		if (idx_frame >= DATA_BUF_SIZE - offset_idx - 1){
			idx_frame = 0;
		}
		if (data_buf_x[idx_frame] != CENTRAL_COORFINATE_X
				&& data_buf_y[idx_frame] != CENTRAL_COORDINATE_Y) {
			data_buf_tran[0] = data_buf_x[idx_frame];
			data_buf_tran[1] = data_buf_y[idx_frame];
			data_buf_tran[2] = data_buf_z[idx_frame];
			CDC_Transmit_FS(data_buf_tran, SEND_BYTE_SIZE);
		}
		//CDC_Transmit_FS(testDataToSending, SEND_BYTE_SIZE);
		idx_frame++;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 3;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffff;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

void DMA2_Stream2_IRQHandler(void){
	if (DMA2->LISR & DMA_LISR_HTIF2){
		DMA2->LIFCR |= DMA_LIFCR_CHTIF2;
	}

	if (DMA2->LISR & DMA_LISR_TCIF2){
		DMA2->LIFCR |= DMA_LIFCR_CTCIF2;
	}
}

uint8_t a = 0;

void DMA2_Stream1_IRQHandler(void){

	if (!COF){
		find_offset(GPIOx_buff);
		COF = 0x1;
	}

	if (DMA2->LISR & DMA_LISR_HTIF1){
		DMA2->LIFCR |= DMA_LIFCR_CHTIF1;
		data_processing(GPIOx_buff, sync_buff, GPIOx_BUF_HALF_SIZE, 0x0, 0x0);
	}

	if (DMA2->LISR & DMA_LISR_TCIF1){
		DMA2->LIFCR |= DMA_LIFCR_CTCIF1;
		data_processing(GPIOx_buff, sync_buff, GPIOx_BUF_HALF_SIZE - offset_idx,
				GPIOx_BUF_HALF_SIZE, DATA_BUF_HALF_SIZE - 1);
	}
}

void TIM8_IRQHandler(void) {

}

void TIM2_IRQHandler(void) {
	if (TIM2->SR & TIM_SR_CC2IF) {
		CNTCurrent = (uint16_t) TIM2->CCR2;
		period = CNTCurrent - CNTBegin;
		pulseWidth = (uint16_t) TIM2->CCR4 - CNTBegin;

		//Calculate duty cycle and save it
		dutyCycle = (float) pulseWidth / (float) period;
		duty_cycle_buff[duty_cycle_idx] = dutyCycle;
		duty_cycle_idx++;
		if (duty_cycle_idx > DC_BUFF_SIZE - 1) {
			duty_cycle_idx = 0;
		}

		CNTBegin = CNTCurrent;
	}
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
