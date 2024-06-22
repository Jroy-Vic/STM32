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

/* MACORS */
#define SET 0x1
#define CLEARED 0x0
#define NEGATIVE -1
#define ARR_SIZE 6		// Size of inputArr

/* MACRO FUNCTIONS */
#define DIGIT_SELECT(x) (((x) == 1) ? 1000 : \
						((x) == 2) ? 100 : \
						((x) == 3) ? 10 : 1) 	// Used to Convert String to Integer

#define CHAR_TO_INT(x) 	((x) - '0')				// Used to Convert CHAR ASCII to Integer

/* Functions */
/* Clears Array, Resets Index, and Resets Terminal */
void reset_terminal(void);

/* Convert String into Integer */
int16_t input_convert(void);

/* Handle Inputs from Terminal */
void input_handler();

/* Check if a Char is a Number */
uint8_t isDigit(char element);

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

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
