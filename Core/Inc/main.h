/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MEM_MOSI_Pin GPIO_PIN_1
#define MEM_MOSI_GPIO_Port GPIOC
#define MEM_MISO_Pin GPIO_PIN_2
#define MEM_MISO_GPIO_Port GPIOC
#define ADDR0_Pin GPIO_PIN_3
#define ADDR0_GPIO_Port GPIOC
#define VIN_VOLTAGE_SENSE_Pin GPIO_PIN_4
#define VIN_VOLTAGE_SENSE_GPIO_Port GPIOA
#define ETH_SCLK_Pin GPIO_PIN_5
#define ETH_SCLK_GPIO_Port GPIOA
#define ETH_MISO_Pin GPIO_PIN_6
#define ETH_MISO_GPIO_Port GPIOA
#define ETH_MOSI_Pin GPIO_PIN_7
#define ETH_MOSI_GPIO_Port GPIOA
#define ADDR1_Pin GPIO_PIN_4
#define ADDR1_GPIO_Port GPIOC
#define ADDR2_Pin GPIO_PIN_5
#define ADDR2_GPIO_Port GPIOC
#define MEM_SCLK_Pin GPIO_PIN_10
#define MEM_SCLK_GPIO_Port GPIOB
#define ADDR3_Pin GPIO_PIN_6
#define ADDR3_GPIO_Port GPIOC
#define ETH_CS_Pin GPIO_PIN_8
#define ETH_CS_GPIO_Port GPIOA
#define ETH_INT_Pin GPIO_PIN_9
#define ETH_INT_GPIO_Port GPIOA
#define ETH_INT_EXTI_IRQn EXTI9_5_IRQn
#define ETH_RST_Pin GPIO_PIN_10
#define ETH_RST_GPIO_Port GPIOA
#define MEM_CS_Pin GPIO_PIN_11
#define MEM_CS_GPIO_Port GPIOC
#define DUMMY_UART_TX_Pin GPIO_PIN_12
#define DUMMY_UART_TX_GPIO_Port GPIOC
#define DUMMY_UART_RX_Pin GPIO_PIN_2
#define DUMMY_UART_RX_GPIO_Port GPIOD
#define SENSE_SDA_Pin GPIO_PIN_6
#define SENSE_SDA_GPIO_Port GPIOB
#define SENSE_SCL_Pin GPIO_PIN_7
#define SENSE_SCL_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
