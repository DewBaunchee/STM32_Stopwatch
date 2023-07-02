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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

extern volatile uint16_t time;
extern int state;
extern int mode;

#define STOPPED 0
#define STARTED 1
#define PAUSED 2

#define DECIMAL 0
#define SECOND 1

#define MODE_COUNT (SECOND + 1)

void setDigit(uint8_t position, uint8_t digit);
void setTime();
void setLed(int stateType);
void setLeds();

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define START_Pin GPIO_PIN_1
#define START_GPIO_Port GPIOA
#define START_EXTI_IRQn EXTI1_IRQn
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define PAUSE_Pin GPIO_PIN_4
#define PAUSE_GPIO_Port GPIOA
#define PAUSE_EXTI_IRQn EXTI4_IRQn
#define LED_A1_Pin GPIO_PIN_5
#define LED_A1_GPIO_Port GPIOA
#define LED_A2_Pin GPIO_PIN_6
#define LED_A2_GPIO_Port GPIOA
#define LED_A3_Pin GPIO_PIN_7
#define LED_A3_GPIO_Port GPIOA
#define CHANGE_Pin GPIO_PIN_0
#define CHANGE_GPIO_Port GPIOB
#define CHANGE_EXTI_IRQn EXTI0_IRQn
#define DS_SHIFT_Pin GPIO_PIN_8
#define DS_SHIFT_GPIO_Port GPIOA
#define DS_DATA_Pin GPIO_PIN_9
#define DS_DATA_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define DS_LATCH_Pin GPIO_PIN_5
#define DS_LATCH_GPIO_Port GPIOB
#define LED_A4_Pin GPIO_PIN_6
#define LED_A4_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
