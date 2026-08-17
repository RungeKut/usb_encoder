/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_PIN_GPIO_Port, LED_PIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, lc_clk_Pin|LED_POW_Pin|LED_220_Pin|dis_res_bp_Pin
                          |dis_rs_bp_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, COMP_ON_Pin|dis_ce_bp_Pin|dis_sel_bp_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_PIN_Pin */
  GPIO_InitStruct.Pin = LED_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_PIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TNG_Pin KEY_reset_Pin */
  GPIO_InitStruct.Pin = TNG_Pin|KEY_reset_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY_ON_Pin KEY_WIN_Pin KEY_PRESET_Pin */
  GPIO_InitStruct.Pin = KEY_ON_Pin|KEY_WIN_Pin|KEY_PRESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : lc_clk_Pin LED_POW_Pin LED_220_Pin dis_res_bp_Pin
                           dis_rs_bp_Pin */
  GPIO_InitStruct.Pin = lc_clk_Pin|LED_POW_Pin|LED_220_Pin|dis_res_bp_Pin
                          |dis_rs_bp_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : COMP_ON_Pin dis_ce_bp_Pin dis_sel_bp_Pin */
  GPIO_InitStruct.Pin = COMP_ON_Pin|dis_ce_bp_Pin|dis_sel_bp_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ENCODER_KEY_Pin */
  GPIO_InitStruct.Pin = ENCODER_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ENCODER_KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY_MUTE_Pin KEY_LEFT_Pin KEY_RIGHT_Pin KEY_PRINTSCREEN_Pin */
  GPIO_InitStruct.Pin = KEY_MUTE_Pin|KEY_LEFT_Pin|KEY_RIGHT_Pin|KEY_PRINTSCREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
