/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_PIN_Pin GPIO_PIN_13
#define LED_PIN_GPIO_Port GPIOC
#define TNG_Pin GPIO_PIN_14
#define TNG_GPIO_Port GPIOC
#define KEY_reset_Pin GPIO_PIN_15
#define KEY_reset_GPIO_Port GPIOC
#define KEY_ON_Pin GPIO_PIN_1
#define KEY_ON_GPIO_Port GPIOA
#define lc_clk_Pin GPIO_PIN_2
#define lc_clk_GPIO_Port GPIOA
#define LED_POW_Pin GPIO_PIN_3
#define LED_POW_GPIO_Port GPIOA
#define LED_220_Pin GPIO_PIN_4
#define LED_220_GPIO_Port GPIOA
#define KEY_WIN_Pin GPIO_PIN_5
#define KEY_WIN_GPIO_Port GPIOA
#define dis_res_bp_Pin GPIO_PIN_6
#define dis_res_bp_GPIO_Port GPIOA
#define dis_rs_bp_Pin GPIO_PIN_7
#define dis_rs_bp_GPIO_Port GPIOA
#define COMP_ON_Pin GPIO_PIN_10
#define COMP_ON_GPIO_Port GPIOB
#define ZYNQ_ON_Pin GPIO_PIN_11
#define ZYNQ_ON_GPIO_Port GPIOB
#define ENCODER_DT_Pin GPIO_PIN_8
#define ENCODER_DT_GPIO_Port GPIOA
#define ENCODER_SW_Pin GPIO_PIN_9
#define ENCODER_SW_GPIO_Port GPIOA
#define ENCODER_KEY_Pin GPIO_PIN_10
#define ENCODER_KEY_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
