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
#include "dac.h"
#include "keypad.h"
#include "math.h"

/* Config. MACROS */
#define ARR_VAL 0x00C7
#define TIM2_NVIC 0
#define TIM2_PRIORITY 0x0
#define TIM2_PRESCALER 0x0

/* LUT_select MACROS */
#define SIN_LUT 0
#define SAW_LUT 1
#define TRI_LUT 2
#define SQR_LUT 4

/* FREQ_select MACROS */
#define ONE_HUNDRED_HZ 1
#define TWO_HUNDRED_HZ 2
#define THREE_HUNDRED_HZ 3
#define FOUR_HUNDRED_HZ 4
#define FIVE_HUNDRED_HZ 5

/* DUTY_select MACROS */
#define TEN_PERCENT 10
#define FIFTY_PERCENT 50
#define NINETY_PERCENT 90
#define HUNDRED_PERCENT 100

/* State Enumerations */
typedef enum {
	INIT_ST, READ_ST, FREQ_ST,
	WAVE_ST, DUTY_ST, SET_ST
} state_t;

/* Global Variables */
#define LUT_CNT 200
static uint8_t LUT_select;				// Selects Waveform Type
static uint16_t FREQ_select;			// Selects Frequency
static uint8_t DUTY_select;				// Selects Duty Cycle
static uint8_t DUTY_cnt;				// Number of points that are set HIGH based on Duty Cycle
static uint16_t LUT_arr[3][LUT_CNT];	// LUT for Sine Wave, Triangle Wave, and Saw Wave

/* LUT Index Variables/MACROS */
static uint8_t LUT_iter = 0;			// LUT Index

/* Math MACROS */
#define PI 3.14159


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);


int main(void)
{
  /* Initialization */

  /* Initialize State for FSM */
  state_t state = INIT_ST;

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Enabling clock for TIM2 (4MHz) */
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

  /* Initialize all configured peripherals */
  /* Initialize Keypad Peripheral */
  keypad_Setup();
  keypad_Reset();
  volatile int8_t keyVal;

  /* Initialize DAC Peripheral */
  DAC_init();

  /* Initialize TIM2 Peripheral */
  TIM2->CR1 &= ~TIM_CR1_CEN;	// Temporarily turn off Timer
  TIM2->PSC = TIM2_PRESCALER;	// No Prescaler (For setup only) [Using 4MHz]
  TIM2->ARR = (uint16_t) ARR_VAL;

  /* Configure NVIC; Set TIM2 interrupts to be highest priority */
  NVIC->IP[TIM2_NVIC] = TIM2_PRIORITY;
  /* Enable NVIC to handle TIM2 interrupts */
  NVIC->ISER[TIM2_NVIC] = (1 << (TIM2_IRQn & 0x1F));

  /* Set Timer Interrupt Conditions */
  TIM2->CR1 &= ~TIM_CR1_UDIS;	// Enable UEVs
  TIM2->DIER |= (TIM_DIER_UIE);	// Enable hardware interrupt for ARR


  /* Main Program */

  while (1)
  {
	  /* FSM Handler */
	  switch (state) {

	  /* INIT_ST: LUT Value Generation and Configuration */
	  case INIT_ST:
		/* Set Default Waveform to be Square Wave, 50% Duty Cycle, 100Hz */
		LUT_select = SQR_LUT;
		FREQ_select = ONE_HUNDRED_HZ;
		DUTY_select = FIFTY_PERCENT;
		DUTY_cnt = (LUT_CNT * ((float) DUTY_select/HUNDRED_PERCENT));

		/* Generation of LUT Values for each Waveform Type */
		/* Sine Wave LUT */
		for (uint8_t i = 0; i < LUT_CNT; i++) {
			double angle = 2 * PI * i / LUT_CNT;
			double sine_value = sin(angle);
			LUT_arr[SIN_LUT][i] = (uint16_t)((sine_value + 1) * 1800);
		}

		/* Saw Tooth LUT */
		for (uint8_t i = 0; i < LUT_CNT; i++) {
			LUT_arr[SAW_LUT][i] = (uint16_t) (i * 1.5);
		}

		/* Triangle LUT */
		for (uint8_t i = 0; i < (LUT_CNT/2); i++) {
			LUT_arr[TRI_LUT][i] = (uint16_t) ((2*i) * 1.5);
		}
		for (uint8_t i = 0; i < (LUT_CNT/2); i++) {
			LUT_arr[TRI_LUT][((LUT_CNT/2) + i)] = (uint16_t) ((2 * ((LUT_CNT/2) - i)) * 1.5);
		}

		/* Enable Timer */
		TIM2->CR1 |= TIM_CR1_CEN;	// Enable counter

		state = SET_ST;
		break;


	  /* READ_ST: Polling for Button Press */
	  case READ_ST:
		  /* Read Keypad */
		  keyVal = keypad_Read();

		  /* If no button is pressed */
		  while (keyVal == NO_BUTTON_PRESS) {
			  /* Output Existing Signal and Continue Polling */
			  keyVal = keypad_Read();
		  }

		  /* If button is pressed */
		  if ((keyVal = keypad_Read()) != NO_BUTTON_PRESS) {
			  if (keyVal >= 1 && keyVal <= 5) {
				  /* Delay for Button Debouncing */
				  for (uint16_t i = 0; i < 10000; i++);

				  /* Change Frequency of Signal */
				  state = FREQ_ST;
			  }
			  else if (keyVal >= 6 && keyVal <= 9) {
				  /* Delay for Button Debouncing */
				  for (uint16_t i = 0; i < 10000; i++);

				  /* Change Waveform Type */
				  state = WAVE_ST;
			  }
			  else if ((keyVal == 0 || keyVal == KEYPAD_STAR || keyVal == KEYPAD_POUND) && (LUT_select == SQR_LUT)) {
				  /* Delay for Button Debouncing */
				  for (uint16_t i = 0; i < 20000; i++);

				  /* Change Duty Cycle */
				  state = DUTY_ST;
			  }
		  }

		  break;


	  /* FREQ_ST: Change Frequency Value */
	  case FREQ_ST:
		  /* Change Iteration-Step Count */
		  switch (keyVal) {
		  case 1:
			  FREQ_select = ONE_HUNDRED_HZ;
			  break;
		  case 2:
			  FREQ_select = TWO_HUNDRED_HZ;
			  break;
		  case 3:
			  FREQ_select = THREE_HUNDRED_HZ;
			  break;
		  case 4:
			  FREQ_select = FOUR_HUNDRED_HZ;
			  break;
		  case 5:
			  FREQ_select = FIVE_HUNDRED_HZ;
			  break;
		  }

		  /* Transition to SET_ST */
		  state = SET_ST;
		  break;


	  /* WAVE_ST: Change Waveform Type */
	  case WAVE_ST:
		  /* Change LUT Data */
		  switch (keyVal) {
		  case 6:
			  LUT_select = SIN_LUT;
			  break;
		  case 7:
			  LUT_select = SAW_LUT;
			  break;
		  case 8:
			  LUT_select = TRI_LUT;
			  break;
		  case 9:
			  LUT_select = SQR_LUT;
			  break;
		  }

		  /* Transition to SET_ST */
		  state = SET_ST;
		  break;


	  /* DUTY_ST: Increment/Decrement Duty Cycle */
	  case DUTY_ST:
		  /* Change Stall Time */
		  switch (keyVal) {
		  case KEYPAD_STAR:
			  if (DUTY_select > TEN_PERCENT) {
				  DUTY_select -= TEN_PERCENT;
				  DUTY_cnt = (LUT_CNT * ((float) DUTY_select/HUNDRED_PERCENT));
			  }
			  break;
		  case 0:
			  DUTY_select = FIFTY_PERCENT;
			  DUTY_cnt = (LUT_CNT * ((float) DUTY_select/HUNDRED_PERCENT));
			  break;
		  case KEYPAD_POUND:
			  if (DUTY_select < NINETY_PERCENT) {
				  DUTY_select += TEN_PERCENT;
				  DUTY_cnt = (LUT_CNT * ((float) DUTY_select/HUNDRED_PERCENT));
			  }
			  break;
		  }

		  /* Transition to SET_ST */
		  state = SET_ST;
		  break;


	  /* SET_ST: Reset Timer and LUT Index */
	  case SET_ST:
		  /* Reset LUT_iter Count */
		  LUT_iter = 0;

		  /* Reset Timer Count */
		  TIM2->EGR |= TIM_EGR_UG;	// Force Update Event to reset timer
		  TIM2->EGR |= ~TIM_EGR_UG;	// Toggle off Force Update Event

		  state = READ_ST;
		  break;
	  }
  }
}



/* Timer Interrupt Handler */
void TIM2_IRQHandler(void) {
	/* When ARR is reached */
	if (TIM2->SR & TIM_SR_UIF) {
		/* Clear interrupt flag */
		TIM2->SR &= ~TIM_SR_UIF;	// Clear UIF Bit to continue polling

		/* Check for LUT Index */
		if (LUT_iter >= LUT_CNT) {
			LUT_iter = 0;
		}

		/* If LUT Index is valid, output signal */
		/* Select Waveform Type */
		switch (LUT_select) {
		case SIN_LUT:
			/* Write Converted Sine Value to DAC */
			DAC_Write(LUT_arr[SIN_LUT][LUT_iter]);

			/* Increment LUT Index by selected frequency */
			LUT_iter += FREQ_select;
			break;

		case SAW_LUT:
			/* Write Converted Saw Value to DAC */
			DAC_Write(DAC_Volt_Conv(LUT_arr[SAW_LUT][LUT_iter]));

			/* Increment LUT Index by selected frequency */
			LUT_iter += FREQ_select;
			break;

		case TRI_LUT:
			/* Write Converted Triangle Value to DAC */
			DAC_Write(DAC_Volt_Conv(LUT_arr[TRI_LUT][LUT_iter]));

			/* Increment LUT Index by selected frequency */
			LUT_iter += FREQ_select;
			break;

		case SQR_LUT:
			/* Write Converted Square Value to DAC */
			if(LUT_iter < DUTY_cnt) {
				/* Write 3.0V if under Duty Cycle threshold point */
				DAC_Write(DAC_Volt_Conv(HIGH_VOLT));
			}
			else {
				/* Write 0V if over Duty Cycle threshold point */
				DAC_Write(LOW_VOLT);
			}

			/* Increment LUT Index by selected frequency */
			LUT_iter += FREQ_select;
			break;
		}
	}
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
