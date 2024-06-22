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

/* MACROS */
#define ARR_SIZE 20			// Sample Array Size
#define CLEARED 0x0			// Zero
#define VOLT_STR_SIZE 5		// Size of String Sent to Terminal (Includes Decimal Point and '\0')
#define ZERO_CHAR '0'		// Zero Character
#define DECIMAL_CHAR '.'	// Decimal Character
#define DECIMAL_IDX 1		// Decimal Index in voltStr
#define NULL_BYTE '\0'		// Null Byte
#define DELAY 100000		// Delay for UART

/* MACRO FUNCTIONS */
#define CALIBRATION(x) ((807 * (x)) - 8970)			// Experimental Calibration Equation [in uV]
#define MICRO_DIVIDE(x) (((x) == 0) ? 1000000 : \
						((x) == 1) ? 100000 : \
						((x) == 2) ? 10000 : 1) 	// Used to check for non-zero digits

/* Global Variables */
uint16_t digital_Val;
uint8_t ISR_Flag;

/* States */
typedef enum {
	POLL_ST, CALC_ST, OUTPUT_ST
} state_t;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* Converts 32-Bit Signed Value to String (Min. Value is 0) */
char* convInt(int32_t val);

int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  /* Initialize ADC1 Peripheral to Channel 1 (PC0) */
  ADC_init();

  /* Initialize UART2 Communication with Terminal Peripheral */
  UART_init();

  /* Create Array to Hold 20 Samples */
  uint16_t digitalArr[ARR_SIZE];
  //clearArr(digitalArr, ARR_SIZE);
  /* Create Array Index */
  uint8_t arrIDX = 0x0;
  /* Declare Value Variables */
  int32_t minVal, maxVal, avgVal;

  /* Initialize FSM */
  state_t state = POLL_ST;

  /* Begin First ADC Sample Conversion */
  ADC_convert();

  while (1)
  {
	  /* Transition States for FSM */
	  switch (state) {
	  /* Polling for ISR_Flag */
	  case POLL_ST:
		  /* Check if Array is Full */
		  /* If Not Full, Save Values to Array */
		  if (arrIDX < ARR_SIZE) {
			  /* Poll for ISR_Flag */
			  while (ISR_Flag == CLEARED);

			  /* Save Converted Value into Array */
			  digitalArr[arrIDX] = digital_Val;

			  /* Reset ISR_Flag */
			  ISR_Flag = CLEARED;

			  /* Increment Index and Sample Again */
			  /* Do not Sample at arrIDX = 19 */
			  if (arrIDX++ < ARR_SIZE) {
				  ADC_convert();
			  }
		  }
		  /* If full, transition to CALC_ST */
		  else {
			  state = CALC_ST;
		  }
		  break;
	  /* Calculate Values using Array */
	  case CALC_ST:
		  /* Define Variables */
		  minVal= digitalArr[0];
		  maxVal = digitalArr[0];
		  avgVal = digitalArr[0];

		  /* Calculate Min., Max., Avg. Values */
		  /* Starting at Second Array Value */
		  for (arrIDX = 1; arrIDX < ARR_SIZE; arrIDX++) {
			  /* Obtain Minimum Value */
			  if (digitalArr[arrIDX] < minVal) {
				  minVal = digitalArr[arrIDX];
			  }
			  /* Obtain Maximum Value */
			  if (digitalArr[arrIDX] > maxVal) {
				  maxVal = digitalArr[arrIDX];
			  }
			  /* Accumulate Values */
			  avgVal += digitalArr[arrIDX];
		  }
		  /* Calculate Average Value */
		  avgVal = (avgVal / ARR_SIZE);

		  /* Calibrate Values to Voltage Values [in uV] */
		  minVal = CALIBRATION(minVal);
		  maxVal = CALIBRATION(maxVal);
		  avgVal = CALIBRATION(avgVal);

		  /* Transition to OUTPUT_ST */
		  state = OUTPUT_ST;
		  break;

	  /* Do Nothing More */
	  case OUTPUT_ST:
		  /* Output Values to Terminal after Converting to String */
		  /* Minimum Value */
		  UART_print("MIN. VALUE: ");
		  UART_print((char*) convInt(minVal));
		  UART_print(" V  |  ");
		  /* Maximum Value */
		  UART_print("MAX. VALUE: ");
		  UART_print((char*) convInt(maxVal));
		  UART_print(" V  |  ");
		  /* Average Value */
		  /* Make Bold (For Fun) */
		  UART_ESC_Code("[1m");
		  UART_print("AVG. VALUE: ");
		  UART_print((char*) convInt(avgVal));
		  UART_print(" V\n\r");
		  /* Reset Text Configuration */
		  UART_ESC_Code("[0m");

		  /* Delay */
		  UART_delay(DELAY);

		  /* Reset Array Index and Sample Again */
		  arrIDX = 0;
		  ADC_convert();
		  state = POLL_ST;
		  break;
	  }
  }
}


/* Converts 32-Bit Signed Value to String (Min. Value is 0) */
char* convInt(int32_t val) {
	/* Declare String Variable, String Index, and Temporary Value */
	static char voltStr[VOLT_STR_SIZE];
	uint8_t strIDX = 0;
	int32_t tempVal;

	/* Check for Length of Value (Not Including Decimal and '\0') */
	for (uint8_t i = 0; i < (VOLT_STR_SIZE - 2); i++) {
		/* Insert Decimal Point in String (x.xx + '\0') */
		if (strIDX == DECIMAL_IDX) {
			voltStr[strIDX++] = DECIMAL_CHAR;
		}
		/* Checks if the First Three Digits are Non-zero and Positive */
		if ((val > 0) && ((tempVal = (val / MICRO_DIVIDE(i))) > 0)) {
			/* Insert Digit in String as ASCII Value */
			voltStr[strIDX++] = (char) (tempVal + ZERO_CHAR);
			/* Remove Leftmost Digit */
			val = (val % MICRO_DIVIDE(i));
		}
		/* If Digit is Zero */
		else {
			/* Insert Zero in String */
			voltStr[strIDX++] = ZERO_CHAR;
			/* Remove Leftmost Digit */
			val = (val % MICRO_DIVIDE(i));
		}
	}

	/* Terminate String with Null-Byte */
	voltStr[strIDX] = NULL_BYTE;

	return voltStr;
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
