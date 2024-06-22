/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "Servo.h"
#include "UART.h"
#include "laser.h"


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize UART Communication with Terminal */
  UART_init();

  /* Initialize Servo Peripherals */
  SERVO_init();

  /* Initialize Laser Peripheral */
  LASER_init();

  /* Set Terminal and Servos to Default State */
  reset_terminal();
  SERVO_activate();

  while (1)
  {
	  /* Display Prompt */
	  UART_print("Input Position Value ($) from 250 to 1250 - [(x/y) , $ , $ , $ , $]: ");

	  /* Wait for Data Input from User or Webcam */
	  while (!(Array_Flag == SET)) {
		  /* Retain Position and Laser State */
		  SERVO_activate();
		  LASER_activate();
	  }
	  /* Clear Array_Flag */
	  Array_Flag = CLEARED;

	  /* Process Data */
	  UART_print("\n\rProcessing Data...\n\r");
	  input_handler();
  }
}


/* Clears Array and Terminal */
void reset_terminal() {
	/* Erase All Current Values */
	for (uint8_t i = 0; i < ARR_SIZE; i++) {
		inputArr[i] = 0;
	}

	/* Reset Index */
	arrIDX = 0;

	/* Reset Terminal */
	UART_ESC_Code("[2J");
	UART_ESC_Code("[H");
}


/* Check if a Char is a Number */
uint8_t isDigit(char element) {
    if (element >= '0' && element <= '9') {
        return SET;
    } else {
        return CLEARED;
    }
}


/* Convert String into Integer */
int16_t input_convert() {
	/* Create Indexer (Skip First Element) */
	uint8_t idx = 1;
	char element;

	/* Create Accumulator */
	uint16_t sum = 0;

	/* Collect Chars */
	while (((element = inputArr[idx]) != NULL_BYTE) && (element != CARRIAGE)
			&& (element != NEW_LINE)) {
		/* Check if Input is Valid */
		if (isDigit(element)) {
			sum += (CHAR_TO_INT(element) * DIGIT_SELECT(idx));
		} else {
			/* Retain Previous Value */
			UART_print("Incorrect Data Format!\n\r");
			return NEGATIVE;
		}

		/* Increment through Array */
		idx++;
	}

	/* Handle Incomplete Inputs */
	if ((element == NEW_LINE) || (element == CARRIAGE)) {
		sum /= DIGIT_SELECT((idx - 1));
	}

	/* If Sum is Less than or Greater than Servo Thresholds, Restrict Values */
	if (sum < MIN_SERVO_POS) {
		sum = MIN_SERVO_POS;
	} else if (sum > MAX_SERVO_POS) {
		sum = MAX_SERVO_POS;
	}

	return sum;
}


/* Handle Inputs from Terminal */
void input_handler() {
	/* Check which Servo the Data Belongs to */
	if ((inputArr[0] == 'x') || (inputArr[0] == 'X')) {
		SERVO_update(input_convert(), NEGATIVE);
	} else if ((inputArr[0] == 'y') || (inputArr[0] == 'Y')) {
		SERVO_update(NEGATIVE, input_convert());
	}
	/* Erase Corrupted Data */
	else {
		UART_print("Incorrect Data Format!\n\r");
	}

	/* Clear Array and Reset Index */
	reset_terminal();
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
