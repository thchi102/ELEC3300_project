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
#include "stm32f1xx_hal.h"

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
#define LCD_Touch_Din_Pin GPIO_PIN_2
#define LCD_Touch_Din_GPIO_Port GPIOE
#define LCD_Touch_Dout_Pin GPIO_PIN_3
#define LCD_Touch_Dout_GPIO_Port GPIOE
#define LCD_Touch_IRQ_Pin GPIO_PIN_4
#define LCD_Touch_IRQ_GPIO_Port GPIOE
#define PN532_SS_Pin GPIO_PIN_0
#define PN532_SS_GPIO_Port GPIOB
#define PN532_IRQ_Pin GPIO_PIN_1
#define PN532_IRQ_GPIO_Port GPIOB
#define PN532_RST_Pin GPIO_PIN_10
#define PN532_RST_GPIO_Port GPIOB
#define LCS_BL_Pin GPIO_PIN_12
#define LCS_BL_GPIO_Port GPIOD
#define LCD_Touch_Select_Pin GPIO_PIN_13
#define LCD_Touch_Select_GPIO_Port GPIOD
#define LCD_Touch_CLK_Pin GPIO_PIN_0
#define LCD_Touch_CLK_GPIO_Port GPIOE
#define LCD_RST_Pin GPIO_PIN_1
#define LCD_RST_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
