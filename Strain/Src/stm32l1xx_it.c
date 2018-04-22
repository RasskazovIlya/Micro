/**
  ******************************************************************************
  * @file    stm32l1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"
#include "stm32l1xx.h"
#include "stm32l1xx_it.h"

/* USER CODE BEGIN 0 */
extern uint8_t button_flag;
uint8_t arr[12], data;
int ch_counter = 0;
uint32_t arr_24[4];

void RREG(int ch_counter);

#define    DWT_CYCCNT    *(volatile unsigned long *)0xE0001004
#define    DWT_CONTROL   *(volatile unsigned long *)0xE0001000
#define    SCB_DEMCR     *(volatile unsigned long *)0xE000EDFC
uint32_t count_tic = 0;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;

/******************************************************************************/
/*            Cortex-M3 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l1xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles RCC global interrupt.
*/
void RCC_IRQHandler(void)
{
  /* USER CODE BEGIN RCC_IRQn 0 */

  /* USER CODE END RCC_IRQn 0 */
  /* USER CODE BEGIN RCC_IRQn 1 */

  /* USER CODE END RCC_IRQn 1 */
}

/**
* @brief This function handles EXTI line2 interrupt.
*/
void EXTI2_IRQHandler(void) //
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */
//	uint8_t wreg = 0x50, data = 0x00, byte1 = 0x00, standby = 0xFF, sync = 0xFC, wakeup = 0x00, rreg = 0x10, rxData = 0x01, rst = 0xFE;
  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_IRQn 1 */
	
//	button_flag = 1;//button is pressed, transmission starts
//	
//	if (button_flag == 1)
//	{
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // CS = 0
//		EXTI->IMR |= EXTI_IMR_MR10; //Mask on EXTI4 (Line 10), DRDY interrupt is off

//		HAL_SPI_Transmit(&hspi1, &rst, 1, 10); //RESET

//		HAL_SPI_Transmit(&hspi1, &sync, 1, 10); //SYNC
//		HAL_SPI_Transmit(&hspi1, &wakeup, 1, 10); //WAKE UP

//		HAL_SPI_Transmit(&hspi1, &rreg, 1, 10); //RREG STATUS first byte
//		HAL_SPI_Transmit(&hspi1, &byte1, 1, 10); //RREG STATUS second byte
//		HAL_SPI_Receive(&hspi1, &rxData, 1, 10);  //RREG STATUS third byte

//		HAL_SPI_Transmit(&hspi1, &wreg, 1, 10); //WREG DRATE first byte
//		HAL_SPI_Transmit(&hspi1, &byte1, 1, 10); //WREG DRATE second byte
//		data = 0xA1; //Data rate = 1000 SpS = 1000 Hz
//		HAL_SPI_Transmit(&hspi1, &data, 1, 10); //WREG DRATE third byte

//		EXTI->IMR &= ~(EXTI_IMR_MR10); //Mask on EXTI4 (Line 10), DRDY interrupt is on
//	}
//	if (button_flag == 0)//if transmission ended, turn off ADC
//	{
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // CS = 1
//		HAL_SPI_Transmit_IT(&hspi1, &standby, 10); //STAND BY
//	}
  /* USER CODE END EXTI2_IRQn 1 */
}

/**
* @brief This function handles EXTI line4 interrupt.
*/
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */
	//uint8_t wreg = 0x51, data = 0x00, byte1 = 0x00, rdata = 0x01, sync = 0xFC, wakeup = 0x00;
  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_IRQn 1 */
	if (button_flag)
	{
		switch (ch_counter)
		{
			case 0:
			{
				RREG(0);
				break;
			}
			case 1:
			{
				RREG(1);
				break;
			}
			case 2:
			{
				RREG(2);
				break;
			}
			case 3:
			{
				RREG(3);
				for(int i = 0; i < 4; i++)
					arr_24[i] = 0;
			
				ch_counter = -1;
				break;
			}
		}
		ch_counter++;
	}
  /* USER CODE END EXTI4_IRQn 1 */
}

/**
* @brief This function handles SPI1 global interrupt.
*/
void SPI1_IRQHandler(void)
{
  /* USER CODE BEGIN SPI1_IRQn 0 */
	
  /* USER CODE END SPI1_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi1);
  /* USER CODE BEGIN SPI1_IRQn 1 */
	
  /* USER CODE END SPI1_IRQn 1 */
}

/**
* @brief This function handles USART1 global interrupt.
*/
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void RREG(int ch_counter)
{
	uint8_t wreg = 0x51, byte1 = 0x00, rdata = 0x01, sync = 0xFC, wakeup = 0x00;
	
	if ((ch_counter + 1) < 4)
		data = 0x01 + (ch_counter + 1)*0x22;
	else data = 0x01;
	
	HAL_SPI_Transmit(&hspi1, &wreg, 1, 10); //WREG MUX channels first byte
	HAL_SPI_Transmit(&hspi1, &byte1, 1, 10); //WREG MUX channels second byte
	HAL_SPI_Transmit(&hspi1, &data, 1, 10); //WREG MUX channels third byte
	
	HAL_SPI_Transmit(&hspi1, &sync, 1, 10); //SYNC
	HAL_SPI_Transmit(&hspi1, &wakeup, 1, 10); //WAKE UP
	
	HAL_SPI_Transmit(&hspi1, &rdata, 1, 10); //RDATA
	
	SCB_DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT_CYCCNT  = 0;
	DWT_CONTROL|= DWT_CTRL_CYCCNTENA_Msk; 
	for (uint32_t i = 0; i < 900000; i++); //delay
	count_tic =  DWT_CYCCNT;
	DWT_CONTROL &= ~DWT_CTRL_CYCCNTENA_Msk; 
	DWT_CYCCNT  = 0;

	HAL_SPI_Receive(&hspi1, &(arr[3*(ch_counter)]), 3, 10); //Data receive from channel ch_counter

	arr_24[ch_counter] += ((uint32_t)(arr[3*(ch_counter)]) << 16) //3x8bit -> 24bit
											 + ((uint32_t)(arr[3*(ch_counter) + 1]) << 8) 
												+ (uint32_t)(arr[3*(ch_counter) + 2]);
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
