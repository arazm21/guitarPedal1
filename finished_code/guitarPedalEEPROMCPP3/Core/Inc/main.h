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
#include "stm32f4xx_hal.h"

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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define interruptButton_Pin GPIO_PIN_6
#define interruptButton_GPIO_Port GPIOA
#define interruptButton_EXTI_IRQn EXTI9_5_IRQn
#define interruptButton2_Pin GPIO_PIN_7
#define interruptButton2_GPIO_Port GPIOA
#define interruptButton2_EXTI_IRQn EXTI9_5_IRQn
#define rotation1_1_Pin GPIO_PIN_4
#define rotation1_1_GPIO_Port GPIOC
#define rotation1_1_EXTI_IRQn EXTI4_IRQn
#define interruptButton3_Pin GPIO_PIN_1
#define interruptButton3_GPIO_Port GPIOB
#define interruptButton3_EXTI_IRQn EXTI1_IRQn
#define I2C_EEPROM_SCL_Pin GPIO_PIN_10
#define I2C_EEPROM_SCL_GPIO_Port GPIOB
#define myButtonPin2_Pin GPIO_PIN_14
#define myButtonPin2_GPIO_Port GPIOB
#define flashLED_Pin GPIO_PIN_15
#define flashLED_GPIO_Port GPIOB
#define interruptLED_Pin GPIO_PIN_10
#define interruptLED_GPIO_Port GPIOA
#define rotation1_2_Pin GPIO_PIN_11
#define rotation1_2_GPIO_Port GPIOA
#define interruptLED2_Pin GPIO_PIN_12
#define interruptLED2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define I2C_EEPROM_SDA_Pin GPIO_PIN_12
#define I2C_EEPROM_SDA_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
