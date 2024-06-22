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
#include "ADC.h"
#include "UART.h"
#include "TIM.h"

/* MACROS */
#define SET 0x1
#define CLEAR 0x0
#define SAMPLE_SIZE 100
#define FCLK 4000000
#define INIT_IDX 0
#define FIVE_PERCENT_LESS 0.95
#define FIVE_PERCENT_MORE 1.05
#define ARR_VAL 0x0190		// 400
#define ARR_INC 0x0004		// 4
#define ARR_FLOOR 0x0028	// 40
#define ARR_CEILING 0x9C40	// 40000
#define FREQ_STR_SIZE 5
#define ZERO_CHAR '0'		// Zero Character
#define NULL_BYTE '\0'		// Null Byte

/* States for FSM */
typedef enum {
	MEAS_ST, CALC_ST, OUTPUT_ST
} state_t;

/* Global Variables */
uint8_t Sample_Flag;
uint8_t ADC_Flag;
uint16_t digital_Val;


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* Check if Target Value is Relatively Close to Reference Value */
uint8_t Value_Checker(uint16_t target, int32_t ref);

/* Converts 32-Bit Signed Value to String (Min. Value is 0) */
char* convInt(int32_t val);


int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure FSM */
  state_t state = OUTPUT_ST;

  /* Create Local Arrays and Variables */
  int32_t valArr[SAMPLE_SIZE];
  uint8_t valIDX = 0;
  uint32_t ARR_val = ARR_VAL;
  uint32_t freq;

  /* Initialize all configured peripherals */
  /* Initialize ADC Peripheral */
  ADC_init();

  /* Initialize UART Communication with Terminal */
  UART_init();

  /* Initialize TIM2 Peripheral */
  TIM_init();

  while (1)
  {
	  /* Handle FSM */
	  switch (state) {

	  /* Fill Sample Array with Measurements */
	  case MEAS_ST:
		  /* Wait for Sample Flag */
		  while (!Sample_Flag);

		  /* Measure Value and Save to Array */
		  ADC_convert();
		  /* Wait for Value */
		  while (!ADC_Flag);
		  /* Clear ADC_Flag */
		  ADC_Flag = CLEAR;
		  valArr[valIDX] = digital_Val;


		  /* If Value Matches Before Completion, Indicates Higher Frequency */
		  /* Check if Value Matches Target Value */
		  if ((valIDX > INIT_IDX) && Value_Checker(digital_Val, valArr[INIT_IDX])) {
			  /* Decrease ARR Value and Reset Sampling */
			  valIDX = CLEAR;
			  /* Set Floor Value at 0x28 */
			  if (ARR_val > ARR_FLOOR) {
				  TIM_setARR((ARR_val - ARR_INC));
			  }

			  /* Start Sampling Again */
			  state = MEAS_ST;
			  break;
		  }

		  /* Check if Array is Full */
		  if (valIDX++ == (SAMPLE_SIZE - 1)) {
			  /* If First and Last Value Match, Frequency Found */
			  if (Value_Checker(digital_Val, valArr[INIT_IDX])) {
				  /* Reset Array and Continue to CALC_ST */
				  valIDX = CLEAR;
				  state = CALC_ST;
				  break;
			  }

			  /* If No Values Match, Indicates Lower Frequency */
			  else {
				  /* Increase ARR Value and Reset Sampling */
				  valIDX = CLEAR;
				  /* Set Ceiling Value at 0x9C40 */
				  if (ARR_val < ARR_CEILING) {
					  TIM_setARR((ARR_val + ARR_INC));
				  }

				  /* Start Sampling Again */
				  state = MEAS_ST;
				  break;
			  }
		  }

		  /* Repeat MEAS_ST */
		  state = MEAS_ST;
		  break;


	  /* Calculate Frequency */
	  case CALC_ST:
		  freq = (FCLK / (ARR_val * SAMPLE_SIZE));

		  /* Transition to OUTPUT_ST */
		  state = OUTPUT_ST;
		  break;


	  /* Output Frequency to Terminal */
	  case OUTPUT_ST:
		  TIM_disable();
		  /* Clear Entire Screen and Start at Top Left */
		  UART_ESC_Code("[2J"); // Clear Screen
		  UART_ESC_Code("[H");  // Move Cursor

		  /* Print Frequency */
		  UART_print("Frequency: ");
		  UART_print((char*) convInt(freq));

		  /* Output Values to Terminal after Converting to String */
		  		  /* Minimum Value */
		  		  UART_print("MIN. VALUE: ");
		  		  UART_print("90");
		  		  UART_print(" V  |  ");
		  		  /* Maximum Value */
		  		  UART_print("MAX. VALUE: ");
		  		  UART_print("90");
		  		  UART_print(" V  |  ");
		  		  /* Average Value */
		  		  /* Make Bold (For Fun) */
		  		  UART_ESC_Code("[1m");
		  		  UART_print("AVG. VALUE: ");
		  		  UART_print("90");
		  		  UART_print(" V\n\r");
		  		  /* Reset Text Configuration */
		  		  UART_ESC_Code("[0m");
		  /* Continue Sampling */
		  state = OUTPUT_ST;
		  break;

	  }
  }
}


/* Check if Target Value is Relatively Close to Reference Value */
uint8_t Value_Checker(uint16_t target, int32_t ref) {
	/* Calculate the upper and lower bounds */
	    int32_t lower_bound = (int32_t)((float)ref * FIVE_PERCENT_LESS);
	    int32_t upper_bound = (int32_t)((float)ref * FIVE_PERCENT_MORE);

	/* Within +/- 5% Margin */
	if ((target >= (uint16_t)lower_bound) && (target <= (uint16_t)upper_bound)) {
			return SET;   // Target value is within the ±5% margin
	} else {
		return CLEAR; // Target value is not within the ±5% margin
	}
}


/* Converts 32-Bit Signed Value to String (Min. Value is 0) */
char* convInt(int32_t val) {
	/* Declare String Variable, String Index, and Temporary Value */
	static char freqStr[FREQ_STR_SIZE];
	uint8_t strIDX = 0;
	char tempChar;

	/* Convert Digits to Chars (Starting from Last Digit) */
	do {
		freqStr[strIDX++] = val % 10 + ZERO_CHAR; // Convert digit to character
		val /= 10;
	} while (val != CLEAR);

	/* Terminate String with Null-Byte */
	freqStr[strIDX] = NULL_BYTE;

	// Reverse the string
	uint8_t Strlen = strIDX;
	for (uint8_t i = 0; i < Strlen / 2; i++) {
		tempChar = freqStr[i];
		freqStr[i] = freqStr[Strlen - i - 1];
		freqStr[Strlen - i - 1] = tempChar;
	}

	return freqStr;
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
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
