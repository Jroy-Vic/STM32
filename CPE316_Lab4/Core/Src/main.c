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
#include "keypad.h"
#define ZEROV 0
#define DELAY 1000
#define INPUT_NUM 3
#define DIGIT1 100
#define DIGIT2 10
#define DIGIT3 1
#define BITMASK 0xFFF
#define DAC_CAL 0x3
#define DAC_SHIFT 0xC
#define MAXVOLT 330
#define MAXBIT 4095


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void DAC_init(void);
uint16_t DAC_volt_conv(uint16_t voltage_in);
void DAC_write(uint16_t conv_volt_in);
uint16_t DAC_Input(void);


int main(void)
{

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize DAC */
  DAC_init();
  DAC_write(ZEROV);

  /* Initialize Keypad */
  keypad_Setup();
  keypad_Reset();

  /* Define Voltage Variable */
  volatile uint16_t testVar;
  volatile int8_t keyVal;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* USER CODE END WHILE */
	  	  keyVal = keypad_Read();
	  	  while (keyVal != KEYPAD_STAR) {
	  		  keyVal = keypad_Read();
	  	  }
	  	  /* Wait for button release */
	  	  while ((keyVal = keypad_Read()) == KEYPAD_STAR);
	  	  /* Write to DAC */
	  	  testVar = DAC_Input();
	  	  testVar = DAC_volt_conv(testVar);
	  	  testVar &= BITMASK;
	  	  testVar |= DAC_CAL << DAC_SHIFT;
	  	  DAC_write(testVar);
	     /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}


void DAC_init(void) {
	/* Enable Clock Register for SPI1 */
		RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
		RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	/* Initialize GPIO for SPI1 for DAC */
	  GPIOA->MODER &= ~(GPIO_MODER_MODE4 | GPIO_MODER_MODE5 | GPIO_MODER_MODE6 | GPIO_MODER_MODE7);
	  GPIOA->MODER |= (GPIO_MODER_MODE4_1 | GPIO_MODER_MODE5_1 |
			  	  	  	  GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1); // Alt. Func. (10)
	  GPIOA->OTYPER &= ~(GPIO_OTYPER_OT4 | GPIO_OTYPER_OT5 | GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7); // PP (0)
	  GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEED4 | GPIO_OSPEEDR_OSPEED5 |
			  	  	  	  GPIO_OSPEEDR_OSPEED6 | GPIO_OSPEEDR_OSPEED7);	// Very High-Speed (11)
	  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD4 | GPIO_PUPDR_PUPD5 | GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD7); // No PU/PD (00) for PA4,6
	  GPIOA->PUPDR |= (GPIO_PUPDR_PUPD5_0 | GPIO_PUPDR_PUPD7_0); // PU (01) for PA5,7
	  GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL4 | GPIO_AFRL_AFSEL5 | GPIO_AFRL_AFSEL6 | GPIO_AFRL_AFSEL7);
	  GPIOA->AFR[0] |= ((GPIO_AFRL_AFSEL4_0 | GPIO_AFRL_AFSEL4_2) | (GPIO_AFRL_AFSEL5_0 | GPIO_AFRL_AFSEL5_2) |
			  	  	  (GPIO_AFRL_AFSEL6_0 | GPIO_AFRL_AFSEL6_2) | (GPIO_AFRL_AFSEL7_0 | GPIO_AFRL_AFSEL7_2)); // AF5 (0101)

	/* Initialize SPI Control Register */
	  SPI1->CR1 &= ~SPI_CR1_BR;		// Baud Rate set to CLK/1 (000)
	  SPI1->CR2 &= ~(SPI_CR2_DS);
	  SPI1->CR2 |= (SPI_CR2_DS_3 | SPI_CR2_DS_1 | SPI_CR2_DS_0); // Data Size to 12 Bits (1011)
	  SPI1->CR1 &= ~SPI_CR1_CPOL;	// Clock Polarity (0)
	  SPI1->CR1 &= ~SPI_CR1_CPHA;	// Clock Phase (0)
	  SPI1->CR1 |= SPI_CR1_MSTR;	// Enable Master Mode
	  SPI1->CR2 |= SPI_CR2_NSSP;	// Set Pulse for CS
	  SPI1->CR2 |= SPI_CR2_SSOE;	// Enable CS Output
	  SPI1->CR1 |= SPI_CR1_SPE;
}


uint16_t DAC_volt_conv(uint16_t voltage_in) {
	/* Function returns the voltage scaled to a 12-bit
	 * value
	 * (4095 = 2^12 = 3.3 V)
	 * and
	 * (0 = 2^0 = 0.0 V)
	 * Note that the voltage in is taken as an integer
	 * i.e. 330 = 3.30V and 000 = 0.00V */
	if (voltage_in > MAXVOLT) {
		return MAXBIT;
	}
	return voltage_in * MAXBIT / MAXVOLT;
}


void DAC_write(uint16_t conv_volt_in) {
	/* Pull the CS Low */
	GPIOA->ODR &= ~GPIO_ODR_OD4;

	/* Wait for Tx to be empty */
	while (!(SPI1->SR & SPI_SR_TXE));

	/* Write 12 Bits to DAC */
	SPI1->DR = conv_volt_in;

	/* Wait for Tx to be empty again and non-busy bus */
	while ((!(SPI1->SR & SPI_SR_TXE)) && (SPI1->SR & SPI_SR_BSY));

	/* Pull the CS High */
	GPIOA->ODR |= GPIO_ODR_OD4;
}


uint16_t DAC_Input() {
	int8_t keyVal;
	uint16_t voltVal = ZEROV;

	/* Read from Keypad */
	for (uint8_t i = 0; i < INPUT_NUM; i++) {
		/* Poll for Input */
		while ((keyVal = keypad_Read()) == NO_BUTTON_PRESS) {
			if (keyVal == KEYPAD_POUND) {
				return ZEROV;
			}
		}

		/* Add keyVal to voltVal */
		switch(i) {
			case 0: voltVal += (keyVal * DIGIT1);
					break;
			case 1: voltVal += (keyVal * DIGIT2);
					break;
			case 2: voltVal += (keyVal * DIGIT3);
					break;
		}

		/* Wait for button release */
		while ((keyVal = keypad_Read()) != NO_BUTTON_PRESS);
	}

	return voltVal;
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
