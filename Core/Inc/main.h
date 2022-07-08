/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

#define PWM_UPDATE_TIM(_ARR, _NUM)  	\
	TIM4->CCR1 = (_NUM);								\
	TIM4->ARR = (_NUM)*3;								\
	TIM1->CCR1 = (_ARR)/2;							\
	TIM1->ARR = (_ARR);
	
	
	
	
typedef struct FREQ_queue_t
{
	uint16_t freq;
	uint16_t pt;
	uint16_t num;
} FREQ_queue_t;
#define FREQ_SAMPLE_NUM_TOTAL		5000	
#define FREQ_QUEUE_SIZE					500
#define rxDMA_bufferSize 12
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void EncoderDMA_NoSynchro(void);
void EncoderDMA_Synchro(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PH0_OSC_IN_Pin GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define PWM_DIR_Pin GPIO_PIN_11
#define PWM_DIR_GPIO_Port GPIOE
#define irq_pin2_Pin GPIO_PIN_8
#define irq_pin2_GPIO_Port GPIOC
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define irq_pin4_Pin GPIO_PIN_2
#define irq_pin4_GPIO_Port GPIOD
#define irq_pin3_Pin GPIO_PIN_3
#define irq_pin3_GPIO_Port GPIOD
#define irq_pin1_Pin GPIO_PIN_5
#define irq_pin1_GPIO_Port GPIOD
#define irq_pin0_Pin GPIO_PIN_6
#define irq_pin0_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
