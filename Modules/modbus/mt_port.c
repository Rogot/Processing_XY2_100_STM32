/**
  ******************************************************************************
  * @file           : mt_port.c
  * @brief          : Additional porting data
  * @author         : MicroTechnics (microtechnics.ru)
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "mt_port.h"
/* Declarations and definitions ----------------------------------------------*/
static uint32_t lockCounter = 0;
UART_HandleTypeDef* modbusUart;
TIM_HandleTypeDef* modbusTimer;
/* Functions -----------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
void EnterCriticalSection()
{
  __disable_irq();
  lockCounter++;
}
/*----------------------------------------------------------------------------*/
void ExitCriticalSection()
{
  lockCounter--;
  if (lockCounter == 0)
  {
    __enable_irq();
  }
}
/*----------------------------------------------------------------------------*/\
void MT_PORT_SetTimerModule(TIM_HandleTypeDef* timer)
{
  modbusTimer = timer;
}
/*----------------------------------------------------------------------------*/\
void MT_PORT_SetUartModule(UART_HandleTypeDef* uart)
{
  modbusUart = uart;
}
/*----------------------------------------------------------------------------*/