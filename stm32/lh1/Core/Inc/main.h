/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RTC_CLK_Pin GPIO_PIN_0
#define RTC_CLK_GPIO_Port GPIOC
#define VBAT_Pin GPIO_PIN_2
#define VBAT_GPIO_Port GPIOC
#define PWM_CH1_Pin GPIO_PIN_0
#define PWM_CH1_GPIO_Port GPIOA
#define PWM_CH2_Pin GPIO_PIN_1
#define PWM_CH2_GPIO_Port GPIOA
#define SOUND_TX_Pin GPIO_PIN_2
#define SOUND_TX_GPIO_Port GPIOA
#define SOUND_RX_Pin GPIO_PIN_3
#define SOUND_RX_GPIO_Port GPIOA
#define SPI1_NSS_Pin GPIO_PIN_4
#define SPI1_NSS_GPIO_Port GPIOA
#define BLE_TX_Pin GPIO_PIN_4
#define BLE_TX_GPIO_Port GPIOC
#define BLE_RX_Pin GPIO_PIN_5
#define BLE_RX_GPIO_Port GPIOC
#define BTN0_Pin GPIO_PIN_0
#define BTN0_GPIO_Port GPIOB
#define BTN1_Pin GPIO_PIN_1
#define BTN1_GPIO_Port GPIOB
#define BTN2_Pin GPIO_PIN_2
#define BTN2_GPIO_Port GPIOB
#define IS_SDB_Pin GPIO_PIN_6
#define IS_SDB_GPIO_Port GPIOC
#define IS_INTB_Pin GPIO_PIN_7
#define IS_INTB_GPIO_Port GPIOC
#define RTC_INT_Pin GPIO_PIN_8
#define RTC_INT_GPIO_Port GPIOC
#define SPI1_NSS1_Pin GPIO_PIN_8
#define SPI1_NSS1_GPIO_Port GPIOA
#define SPI1_NSS2_Pin GPIO_PIN_15
#define SPI1_NSS2_GPIO_Port GPIOA
#define BTN3_Pin GPIO_PIN_3
#define BTN3_GPIO_Port GPIOB
#define BTN4_Pin GPIO_PIN_4
#define BTN4_GPIO_Port GPIOB
#define DBG_TX_Pin GPIO_PIN_6
#define DBG_TX_GPIO_Port GPIOB
#define DBG_RX_Pin GPIO_PIN_7
#define DBG_RX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
