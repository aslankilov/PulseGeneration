/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "string.h"
#include "binary_support.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define M_PI 3.141592653589

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t test_command = 0;		//variable testing commands: 
static volatile uint8_t UART_DMA_Test[10] = {0xAF, 1, 2, 3, 4, 5, 6, 7, 8, 9};

//--------------------------------------------------------
int32_t FREQ_UART_BUFFER[FREQ_SAMPLE_NUM_TOTAL] = {0};
static uint8_t UART_DMA_TxBuf[800+2] = {0};
static uint8_t UART_DMA_TxCommand[7] = {0};
uint8_t FREQ_DATA_SEND = 0, FREQ_START = 0, FREQ_DATA_SEND_OK = 1;
extern uint16_t freq_queue_pt; 
volatile uint32_t command_transmit_error_num = 0, data_transmit_error_num = 0, freq_queue_overrun_count = 0;


volatile FREQ_queue_t FREQ_queue[FREQ_QUEUE_SIZE]; 
volatile uint8_t FREQ_UART_Rx = 0;
volatile uint8_t FREQ_uart_received = 0;
uint16_t freq_queue_pt_local, crc, freq_loc, num_loc, pt_loc;
uint32_t total_pack_num = 0;



int32_t intPos = 0;
int32_t intPosDelta = 0; 
int32_t intPosPrev = 0;
int32_t intPosCurrent = 0;
//-----------------------------------------------------------------------


uint32_t TransmitTickCount = 0;

uint8_t ring_buf_first = 0;
uint8_t pack[6] = {0};
uint8_t rec_count = 0;
uint8_t state = 0;

uint16_t checkError = 0;

void (*EncoderHandler)(void);
HAL_StatusTypeDef status_check;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;		// разрешаем использовать DWT
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; 			// включаем счётчик
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
  MX_TIM3_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	EncoderHandler = EncoderDMA_NoSynchro;
	
	HAL_UART_Receive_DMA(&huart1, &ring_buf_first, 1);
	
	TIM1_Init();
	TIM4_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	test_command = 0;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if((freq_queue_pt != freq_queue_pt_local))//(freq_queue_pt != 0)
    {
			//DWT->CYCCNT = 0;
        while(!(huart2.gState == HAL_UART_STATE_READY));
        
        freq_loc = FREQ_queue[freq_queue_pt_local].freq;
        num_loc = 4*FREQ_queue[freq_queue_pt_local].num;
				total_pack_num += FREQ_queue[freq_queue_pt_local].num;
        pt_loc = FREQ_queue[freq_queue_pt_local].pt;
        freq_queue_pt_local++;
				freq_queue_pt_local %= FREQ_QUEUE_SIZE;
				if(freq_loc == 0)
				{
					 Error_Handler();
				}
        UART_DMA_TxCommand[0] = 0xAF;
				UART_DMA_TxCommand[1] = (freq_loc & 0xFF00)>>8;
				UART_DMA_TxCommand[2] = (freq_loc & 0x00FF);
				UART_DMA_TxCommand[3] = ((num_loc) & 0xFF00)>>8;
				UART_DMA_TxCommand[4] = ((num_loc) & 0x00FF);
				crc = calcCRC((uint8_t*)&UART_DMA_TxCommand[1], 4);
				UART_DMA_TxCommand[5] = (crc & 0xFF00)>>8;
				UART_DMA_TxCommand[6] = (crc & 0x00FF);
				// Отправка пакета команды
				status_check = HAL_UART_Transmit_DMA(&huart2, (uint8_t* )UART_DMA_TxCommand, 7);
				
        memcpy((void *)&UART_DMA_TxBuf[0], (void *)&FREQ_UART_BUFFER[pt_loc], num_loc);
        FREQ_DATA_SEND_OK = 0;
				FREQ_uart_received = 0;
				FREQ_UART_Rx = 0;
				while(!FREQ_DATA_SEND_OK)
				{
					while(!FREQ_uart_received);
					
					if(FREQ_UART_Rx == 0x12)
					{
							FREQ_DATA_SEND_OK = 1;
							FREQ_UART_Rx = 0;
					}
					else //if(FREQ_UART_Rx == 0x14)
					{
						command_transmit_error_num++;
						FREQ_uart_received = 0;
						FREQ_UART_Rx = 0;
						status_check = HAL_UART_Transmit_DMA(&huart2, (uint8_t* )UART_DMA_TxCommand, 7);
					}
				}
        crc = calcCRC((uint8_t*)&UART_DMA_TxBuf[0], num_loc);
        UART_DMA_TxBuf[num_loc] = (crc & 0xFF00)>>8;
        UART_DMA_TxBuf[num_loc + 1] = (crc & 0x00FF);
        FREQ_DATA_SEND_OK = 0;
				
        do
        {
					FREQ_uart_received = 0;
					FREQ_UART_Rx = 0;
					status_check = HAL_UART_Transmit_DMA(&huart2, (uint8_t* )UART_DMA_TxBuf, num_loc + 2);
					while(!FREQ_uart_received);
							
					if(FREQ_UART_Rx == 0x43)
					{
							FREQ_DATA_SEND_OK = 1;
					}
					else if(FREQ_UART_Rx == 0x14)
					{
						data_transmit_error_num++;
//							while(1);
					}
        }while(!FREQ_DATA_SEND_OK);
				//TransmitTickCount = DWT->CYCCNT;
    }
		
		if(test_command == 7)
		{
			test_command = 0;
			FREQ_START = 1;
			HAL_TIM_Base_Start_IT(&htim3);// 5kHz
			TIM6->EGR = TIM_EGR_UG;
			TIM6->SR = 0;
			__HAL_TIM_ENABLE_IT(&htim6, TIM_IT_UPDATE);
			HAL_TIM_Base_Start_IT(&htim6);// 4kHz			
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


uint8_t calculated_check;

void EncoderDMA_NoSynchro(void)
{
		pack[rec_count] = ring_buf_first;
		if(!state && (ring_buf_first == 0x32))
		{
			state = 1;
		}
		if(state)
		{
			rec_count++;
			if(rec_count >= 6)
			{
				rec_count = 0;
				state = 0;
				EncoderHandler = EncoderDMA_Synchro;
				CLEAR_BIT(DMA2_Stream2->CR, DMA_SxCR_EN | DMA_SxCR_HTIE);
				DMA2_Stream2->M0AR = (uint32_t) pack;
				DMA2_Stream2->NDTR = 6;
				SET_BIT(DMA2_Stream2->CR, DMA_SxCR_MINC);
				SET_BIT(DMA2_Stream2->CR, DMA_SxCR_EN);
				calculated_check = pack[0] ^ pack[1] ^ pack[2] ^ pack[3] ^ pack[4];
				if (calculated_check == pack[5])
				{
					intPos = ((uint32_t)pack[2]) | ((uint32_t)pack[3]<<8) | ((uint32_t)pack[4]<<16);		
					intPosDelta = (intPos - intPosPrev);// (intPos - intPosPrev); 
					intPosPrev = intPos;
					intPosDelta = intPosDelta - (intPosDelta/2560000) * (5120000-1);
					intPosCurrent += intPosDelta;
				}
				else
				{
					checkError += 1;
				}
			}
		}
}

void EncoderDMA_Synchro(void)
{
	calculated_check = pack[0] ^ pack[1] ^ pack[2] ^ pack[3] ^ pack[4];
	if (calculated_check == pack[5])
	{
		intPos = ((uint32_t)pack[2]) | ((uint32_t)pack[3]<<8) | ((uint32_t)pack[4]<<16);	
		intPosDelta = (intPos - intPosPrev);// (intPos - intPosPrev); 
		intPosPrev = intPos;
		intPosDelta = intPosDelta - (intPosDelta/2560000) * (5120000-1);
		intPosCurrent += intPosDelta;
	}
	else
	{
		checkError += 1;
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
