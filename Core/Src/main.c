/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hmi_interface.h"

#if MODBUS_ENABLE
#include "mb.h"
#include "mbport.h"
#include "mt_port.h"
#endif

#if DWIN_SERIAL_PORT_ENABLE
#include "DWIN_lib.h"
#include "DWIN_port.h"
#endif

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
extern t_control ctrl;
extern t_hmi_reg programs[10];
extern uint8_t start;
extern uint8_t print;
extern t_step_engine step_engine;

#if MODBUS_ENABLE
extern t_modbus_him_pack rxData;
extern t_modbus_him_pack txData;

extern uint16_t rx_data[256];
extern uint8_t rx_data_indx;

static USHORT usRegHoldingStart = REG_HOLDING_START;
static int usRegHoldingBuf[REG_HOLDING_NREGS] = { 0x0 };
	
static USHORT usRegInputStart = REG_INPUT_START;
static USHORT usRegInputBuf[REG_INPUT_NREGS] = { 0x0 };
#endif

#if DWIN_SERIAL_PORT_ENABLE
extern volatile USHORT  ucRegistersBuf[DWIN_SER_PDU_SIZE_MAX];
extern volatile UCHAR  ucDWINBuf[DWIN_SER_PDU_SIZE_MAX];
#endif 
extern uint16_t in_count;
t_dac dac;
uint8_t is_start_pos = 0x0;

t_devices dev;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM10_Init(void);
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
	#if MODBUS_ENABLE
	
	MT_PORT_SetTimerModule(&htim10);
	MT_PORT_SetUartModule(&huart1);
	
	eMBErrorCode eStatus;
	//eStatus = eMBInit(MB_RTU, 0x0A, 0, 19200, MB_PAR_NONE);
	eStatus = eMBInit(MB_RTU, 0x01, 0, 115200, MB_PAR_NONE);
	eStatus = eMBEnable();
	if (eStatus != MB_ENOERR)
	{
	// Error handling
	}
	
	usRegHoldingBuf[NUM_EXE_PROGRAM] = 0x01;
	usRegHoldingBuf[STEP_ENGINE_VEL_MC] = 0x30;
	usRegHoldingBuf[STEP_ENGINE_START_POS_MS] = 0x51;
	
	usRegHoldingBuf[STAGE_1_POS] = 3600;
	usRegHoldingBuf[STAGE_1_VEL] = 360;
	
	#endif
	
	#if PEREPH_ENABLE
	dev.step_engine = &step_engine;
	
	ctrl.dev = &dev;
	ctrl.programms = programs;
	
	init_HMI(&ctrl);
	#endif
	
	#if STEP_ENGINE_ENABLE
	//HAL_DBGMCU_EnableDBGStandbyMode();
  //HAL_DBGMCU_EnableDBGStopMode();
	//DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM3_STOP;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
	
	step_engine.mode = STOP;
	step_engine.vel = SPEED_MIN;
	step_engine.accel_size = 0;
	step_engine.dir = 1;
	step_engine.engine_TIM_master = &htim3;
	step_engine.engine_TIM_slave = &htim2;
	
	init_step_engine(&step_engine);
	#endif
	
	uint8_t count = 0;
	
	#if DAC_ENABLE
	/* DAC init */
	dac.tim = TIM6;
	dac.dac_type = DAC;
	dac.tim_presc = 719;
	dac.tim_arr = 10;
	CMSIS_DAC_init(&dac);
	uint16_t a;
	#endif
	
	#if DWIN_SERIAL_PORT_ENABLE
	//DWIN_PORT_SetDMAModule(DMA2_Stream2);
	DWIN_PORT_SetUartModule(&huart1);
	//CMSIS_DMA_Init(DMA2_Stream7);
	//CMSIS_DMA_Init(DMA2_Stream2);
	
	//HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);                                        
	//HAL_NVIC_EnableIRQ(USART1_IRQn);
	
	eDWINErrorCode eStatus;
	eDWINInit(0xA5, 0, 115200, DWIN_PAR_NONE); 
	
	eStatus = eDWINEnable();
	if (eStatus != DWIN_ENOERR)
	{
	// Error handling
	}
	#endif
	
	//HAL_UARTEx_ReceiveToIdle_IT(&huart1, (uint8_t *)ucDWINBuf, DWIN_SER_PDU_SIZE_MAX);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		#if PEREPH_ENABLE
			#if DATA_USART2_TX
			cnt++;
			if (cnt >= 255) cnt = 0;
			DataWrite(cnt);
			HAL_Delay(10);
			#endif		
		
			#if DAC_ENABLE
			//dac.dac_type->DHR12R1 = a;
			//a++;
			#endif
		
			#if MODBUS_ENABLE
			eHMIPoll(&ctrl, usRegHoldingBuf);
			#endif
		
			#if DWIN_SERIAL_PORT_ENABLE
			eHMIPoll(&ctrl, ucRegistersBuf);
			#endif
		#endif
		
		#if MODBUS_ENABLE
		eMBPoll();
		#endif
		
		#if DWIN_SERIAL_PORT_ENABLE
		eDWINPoll();
		#endif
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 71;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 50;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
