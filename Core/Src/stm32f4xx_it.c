/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define M_PI 3.141592653589
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
const uint32_t PWM_DIR_STATE[2] = {PWM_DIR_Pin<<16, PWM_DIR_Pin};
static volatile uint16_t freq_buf_pt = 0;
static volatile uint8_t next_pack = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//extern volatile uint8_t FREQ_UART_Rx;
//extern volatile uint8_t FREQ_uart_received;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;
extern DMA_HandleTypeDef hdma_usart2_tx;
/* USER CODE BEGIN EV */
extern DMA_HandleTypeDef hdma_usart2_tx;  

int32_t intPosGoal, intPosGoal_32, last_intPosGoal;
int32_t error_position;


uint16_t freq_send_num = 0, freq_start_pt = 0, freq_base_current = 1, 
					freq_base_prev = 1, freq_queue_pt = 0, freq_base = 1, freq_base_period = 1;

static uint16_t count_step = 1, count = 0, count_max = 10000, period = 0, period_max = 5;

extern int32_t intPosCurrent;

/* -------------------- FREQ_UART variables ------------------- */
extern uint8_t FREQ_START;
extern volatile FREQ_queue_t FREQ_queue[FREQ_QUEUE_SIZE];
extern uint32_t freq_queue_overrun_count;
extern volatile uint8_t FREQ_UART_Rx;
extern volatile uint8_t FREQ_uart_received;
extern int32_t FREQ_UART_BUFFER[FREQ_SAMPLE_NUM_TOTAL];
/* ------------------------------------------------------------ */


extern void (*EncoderHandler)(void);

extern uint32_t TickCount;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
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
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
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
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream6 global interrupt.
  */
void DMA1_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */
	if(READ_BIT(DMA1->HISR, DMA_HISR_TCIF6))
	{
		huart2.gState = HAL_UART_STATE_READY;
	}
  /* USER CODE END DMA1_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */

  /* USER CODE END DMA1_Stream6_IRQn 1 */
}


/* USER CODE BEGIN 1 */

void TIM4_IRQHandler(void)
{
	TIM1->CCR1 = 0;
	TIM4->CCR1 = 0;
	TIM4->CNT = TIM1->CNT = 0;
	TIM4->SR = 0;
	next_pack = 1;
}


uint16_t pulse_num = 1, arr = 0, pulse_count = 0;
uint32_t total_pulse = 0, total_packet_num = 0;
void TIM6_DAC_IRQHandler(void)
{
	
	irq_pin0_GPIO_Port->BSRR = irq_pin0_Pin;	
	TIM6->SR = ~TIM_SR_UIF;
	if(FREQ_START)
	{
		error_position = intPosGoal - last_intPosGoal;

//		DWT->CYCCNT = 0;
		if(error_position > 0)
		{
			arr = 33600/(error_position+1);
			while(!next_pack);
			next_pack = 0;
			PWM_DIR_GPIO_Port->BSRR = PWM_DIR_STATE[1];
			PWM_UPDATE_TIM(arr, error_position);
			total_pulse += error_position;
//			TickCount = DWT->CYCCNT;
		}
		else if(error_position < 0)
		{
			error_position *= -1;
			arr = 33600/error_position;
			while(!next_pack);
			next_pack = 0;
			PWM_DIR_GPIO_Port->BSRR = PWM_DIR_STATE[0];
			PWM_UPDATE_TIM(arr, error_position);
			total_pulse += error_position;
		}
		else
		{
			arr = 0;
		}

		freq_base_current = freq_base;
			
			
		if((freq_base_current != freq_base_prev) && (freq_buf_pt%200))
		{
				FREQ_queue[freq_queue_pt].freq = freq_base_prev;
				if(freq_base_prev == 0)
				{
						Error_Handler();
				}
				FREQ_queue[freq_queue_pt].pt = freq_buf_pt - freq_buf_pt%200;
				FREQ_queue[freq_queue_pt].num = freq_buf_pt%200;
				total_packet_num += 1;
				freq_queue_pt += 1;
				freq_buf_pt += 200 - freq_buf_pt%200;
				if(freq_buf_pt >= FREQ_SAMPLE_NUM_TOTAL)
				{
						freq_buf_pt = 0;
				}
		}
		freq_base_prev = freq_base_current;

		FREQ_UART_BUFFER[freq_buf_pt] = intPosGoal_32;
		FREQ_UART_BUFFER[freq_buf_pt+1] = intPosCurrent;
		
		freq_buf_pt += 2;
		
		if(freq_queue_pt >= FREQ_QUEUE_SIZE)
		{
			freq_queue_pt = 0;
			freq_queue_overrun_count++;
		}
		
		if(!(freq_buf_pt%200))
		{	
				FREQ_queue[freq_queue_pt].freq = freq_base_current;
				if(freq_base_current == 0)
				{
						Error_Handler();
				}
				FREQ_queue[freq_queue_pt].pt = freq_buf_pt - 200;
				FREQ_queue[freq_queue_pt].num = 200;
				freq_queue_pt += 1;
				total_packet_num += 1;
		}

		if(freq_buf_pt >= FREQ_SAMPLE_NUM_TOTAL)
		{
				freq_buf_pt = 0;
		}
			
	}

	
	irq_pin0_GPIO_Port->BSRR = irq_pin0_Pin<<16;
}


void TIM3_IRQHandler(void)
{
  
	irq_pin2_GPIO_Port->BSRR = irq_pin2_Pin;
	TIM3->SR = ~TIM_SR_UIF;
	intPosGoal = 600*sinf(2*M_PI*(count*1.0f)/count_max);
	intPosGoal_32 = 128000*sinf(2*M_PI*(count*1.0f)/count_max);

	period += (count/count_max);
	count += count_step;
	count %= (count_max + count_step);

	if(period/period_max)
	{
		period = 0;
		period_max += 5*(freq_base_period/20);
		freq_base += 1;
		freq_base_period %= 20;
		freq_base_period++;
		
		count_step = freq_base;
		
		count_max = 10000 + (freq_base - 10000%freq_base);
		
		CLEAR_BIT(TIM3->CR1, (TIM_CR1_CEN * (freq_base/71)));
		FREQ_START = (READ_BIT(TIM3->CR1, TIM_CR1_CEN));
	}
	irq_pin2_GPIO_Port->BSRR = irq_pin2_Pin<<16;
}




void USART2_IRQHandler(void)
{
	
	if(READ_BIT(USART2->SR, USART_SR_RXNE))
	{
		FREQ_UART_Rx = (uint8_t)READ_REG(USART2->DR);
		FREQ_uart_received = 1;
	}
	else if(READ_BIT(USART2->SR, USART_SR_TC))
	{
		CLEAR_BIT(USART2->SR, USART_SR_TC);
	}

}

void DMA2_Stream2_IRQHandler(void)
{
	irq_pin3_GPIO_Port->BSRR = irq_pin3_Pin;
	if(READ_BIT(DMA2->LISR, DMA_LISR_TCIF2))
	{
		EncoderHandler();
		DMA2->LIFCR = 0x300000;
	}
	irq_pin3_GPIO_Port->BSRR = irq_pin3_Pin<<16;
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
