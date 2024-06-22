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
#define ARR_VAL 0x0320
#define CCR_VAL 0x00c8
#define TIM2_NVIC 0
#define TIM2_PRIORITY 0x0
#define TIM2_PRESCALER 0x0


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void TIM2_IRQHandler(void);
/* USER CODE END PFP */


int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

  /* Enabling clock for TIM2 (4MHz) */
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;


  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* Initialize Output Pin */
  GPIOC->MODER &= ~GPIO_MODER_MODE0;
  GPIOC->MODER |= GPIO_MODER_MODE0_0;	// Output (01)
  GPIOC->OTYPER &= ~GPIO_OTYPER_OT0;	// PP (0)
  GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED0;	// Low Speed (00)
  GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD0;
  GPIOC->PUPDR |= GPIO_PUPDR_PUPD0_1;	// PD (10)

  /* Initialize Output as High */
  GPIOC->ODR |= GPIO_ODR_OD0;

  /* Initialize Timer */
  TIM2->CR1 &= ~TIM_CR1_CEN;	// Temporarily turn off Timer
  TIM2->PSC = TIM2_PRESCALER;	// No Prescaler (For setup only)
  TIM2->ARR = (uint16_t) ARR_VAL;
  TIM2->CCR1 = (uint16_t) CCR_VAL;

  /* Set TIM2 interrupts to be highest priority */
  NVIC->IP[TIM2_NVIC] = TIM2_PRIORITY;
  /* Enable NVIC to handle TIM2 interrupts */
  NVIC->ISER[TIM2_NVIC] = (1 << (TIM2_IRQn & 0x1F));

  /* Set Timer Conditions and Enable */
  TIM2->CR1 &= ~TIM_CR1_UDIS;	// Enable UEVs
  TIM2->DIER |= (TIM_DIER_UIE | TIM_DIER_CC1IE);	// Enable hardware interrupt
  TIM2->CR1 |= TIM_CR1_CEN;	// Enable timer
  TIM2->EGR |= TIM_EGR_UG;	// Force Update Event to reset timer
  TIM2->EGR |= ~TIM_EGR_UG;	// Toggle off Force Update Event


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  ; // Wait for Interrupt to occur
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* Timer Interrupt Handler */
void TIM2_IRQHandler(void) {
	/* If ARR is reached */
	if (TIM2->SR & TIM_SR_UIF) {
		GPIOC->ODR |= GPIO_ODR_OD0;	// Output High LED
		TIM2->SR &= ~ TIM_SR_UIF;	// Clear UIF Bit to continue polling
	}
	/* Else if CCR1 is reached */
	else if (TIM2->SR & TIM_SR_CC1IF){
		GPIOC->ODR &= ~GPIO_ODR_OD0;	// Output Low LED
		TIM2->SR &= ~TIM_SR_CC1IF;	// Clear CC1IF Bit to continue polling
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
