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
#include "math.h"
#include "stdlib.h"
//#include "stm32l4xx_hal_uart.h"

////#define ARM_MATH_CM4
//#define ARRAY_SIZE 2048
//static uint16_t data;
//static uint8_t flag = 0;
//static uint16_t conversions[ARRAY_SIZE];
//static uint16_t j = 0, k=0;
//UART_HandleTypeDef huart2;
/* USER CODE BEGIN PV */
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
//static void MX_GPIO_Init(void);
//static void MX_USART2_UART_Init(void);
//char *freq_to_string(uint32_t num, char* string, uint32_t str_len);
//void ConfigureTimer(void);
//void ConfigureGPIO();
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
/**
* @brief  The application entry point.
* @retval int
*/
//static uint8_t freq_flag = 0;
//static uint32_t freq = 0;
//static uint32_t freq=0;
//static uint8_t freq_flag = 0;
//volatile uint32_t a,c;
//volatile uint32_t b[2];
//static uint32_t point1set = 0,point1 = 0,point2 = 0;
//uint8_t flipper;
//uint32_t cnter = 0;
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
/* USER CODE BEGIN SysInit */
/* USER CODE END SysInit */
/* Initialize all configured peripherals */
//MX_GPIO_Init();
//MX_USART2_UART_Init();
UART_init();
/* USER CODE BEGIN 2 */
/* USER CODE END 2 */
/* Infinite loop */
/* USER CODE BEGIN WHILE */
//ADC_init();
//ConfigureGPIO();
//ConfigureTimer();
//ConfigureInterrupts();
while (1) {
	UART_print("MIN. VALUE: ");
}
//ADC_convert();
//char string1[500];
//  for(int i =0; i < 500; i++){
//	  string1[i] = 0;
//  }
//  UART_ESC_Code("[?25l");
//while (1)
//{
//	  if(flag) {
//	  		  //data = calibration(data);
//		  if(j<ARRAY_SIZE){
//	  	   	  conversions[j] = data;
//	  		  j++;
//		  }
//		  flag = 0;
//		  ADC1->CR |= ADC_CR_ADSTART;
//	  }
//	  if(j==ARRAY_SIZE){
//		  uint32_t ave = 0;
//		  for (uint16_t i = 0; i < ARRAY_SIZE; i++)
//			  ave += conversions[i];
//		  ave /= ARRAY_SIZE;
//		  uint64_t square = 0;
//		  float mean;
//		  uint32_t root;
//		  // Calculate square.
//		  for (uint16_t i = 0; i < ARRAY_SIZE; i++) {
//			  square += conversions[i]*conversions[i];
//		  }
//		  // Calculate Mean.
//		  mean = (square /(float)ARRAY_SIZE);
//		  // Calculate Root.
//		  root = (uint32_t)sqrtf(mean);
//		  uint32_t vpp = sqrt(((uint64_t)(root*root)-(ave*ave))*2)*2;
//		  //vpp = ave_calibration(vpp);
//		  char vppString[5];
//		  //int_to_string(vpp, vppString);
//		  //root = ave_calibration(root);
//		  char rootString[5];
//		  //int_to_string(root, rootString);
//		  //ave = ave_calibration(ave);
//		  char aveString[5];
//		  //int_to_string(ave, aveString);
//		  for(int i=0; i<5000000;i++);
//		  UART_ESC_Code("[2J");
//		  UART_print("\r\n");
//		  UART_print("DC Voltage: ");
//		  UART_print(aveString);
//		  UART_print("V\r\n");
//		  for(int8_t i = 0; i<((ave/100000)+1); i++) {
//			  UART_print("#");
//		  }
//		  UART_print("\r\n|----|----|----|----|----|----|\r\n");
//		  UART_print("0   0.5  1.0  1.5  2.0  2.5  3.0\r\n\n");
//		  UART_print("AC Vpp: ");
//		  UART_print(vppString);
//		  UART_print("V\r\n");
//		  UART_print("AC RMS: ");
//		  UART_print(rootString);
//		  UART_print("V\r\n");
//		  for(int8_t i = 0; i<((root/100000)+1); i++) {
//			  UART_print("#");
//		  }
//		  UART_print("\r\n|----|----|----|----|----|----|\r\n");
//		  UART_print("0   0.5  1.0  1.5  2.0  2.5  3.0\r\n\n");
//		  char buffer [33];
//		  		  	  		  			  itoa(24000000/abs(point1-point2),buffer,10);
//		  		  	  		  			  UART_print("Frequency: ");
//		  		  	  		  			  UART_print(buffer);
//		  		  	  		  			  UART_print("Hz\r\n");
//		  /*if(freq_flag) {
//		  	  		  		  	  freq *= 2;
//		  	  		  			  b[flipper & 1] = freq - a;
//		  	  		  			  char* str = freq_to_string(120000000 / (freq - a), string1, 200);
//		  	  		  			  cnter = 0;
//		  	  		  			  flag = 0;
//		  	  		  			  if ((freq - a != 0) && (b[flipper & 1] == b[(~flipper) & 1] || cnter > 1000)) {
//		  	  		  				  if (120000000 / (freq - a) > 0) {
//		  	  		  					  //USART_ESC_Code("[1;1H");
//		  	  		  					  //USART_ESC_Code("[2K");
//		  	  		  					  //UART_print("Hz\r\n");
//		  	  		  					  //UART_print(str);
//		  	  		  					  //UART_print("Frequency: ");
//		  	  		  				  }
//		  	  		  				  flipper++;
//		  	  		  			  }
//		  	  		  			  a = freq;
//		  	  		  		  }
//		  	  		  		  cnter++;
//		  	  		  	char buffer [33];
//		  	  		  			  itoa(freq,buffer,10);
//		  	  		  			  UART_print("Frequency: ");
//		  	  		  			  UART_print(buffer);
//		  	  		  			  UART_print("Hz\r\n");*/
//		  j = 0;
//	  }
//	  /*if(flag && k<FFT_BUFFER_SIZE) {
//		  flag = 0;
//		  ADC1->CR |= ADC_CR_ADSTART;
//	  }*/
//}
///* USER CODE END 3 */
//}
//void ADC1_2_IRQHandler(void) {
//	if(ADC1->ISR & ADC_ISR_EOC){
//		data = ADC1->DR; //read data
//		flag = 1;
//	}
//}
//void TIM2_IRQHandler(void) {
//	freq_flag = 1;
//	if(!point1set){
//		point1 = TIM2->CCR1;
//		point1set = 1;
//	}
//	else {
//		//freq -= TIM2->CCR1;
//		point2 = TIM2->CCR1;
//		point1set = 0;
//	}
//	/*if(!point1set) {
//		point1 = TIM2->CCR1;
//		point1set = 1;
//	}
//	else{
//		freq = point1-freq;
//		point1set = 0;
//	}*/
// // check update interrupt flag
// if (TIM2->SR & TIM_SR_CC1IF) {
// 	// clear flag
// 	//TIM2->SR &= ~(TIM_SR_CC1IF | TIM_SR_CC1OF);
// 	//GLOABL_VAL +=1;
// 	//freq = TIM2->CCR1;
//     // toggle led
//     //TIM2->CCR1 = TIM2->CNT + 399;
// 	//GPIOA->ODR ^= GPIO_ODR_OD5;
// 	TIM2->SR &= ~(TIM_SR_CC1IF);
// }
}
void ConfigureTimer(void) {
 // enable clock
 RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
 TIM2->CCMR1 = 1; //active input
 TIM2->CCMR1 |= TIM_CCMR1_IC1F_0 | TIM_CCMR1_IC1F_1 | TIM_CCMR1_IC1F_2 | TIM_CCMR1_IC1F_3  ; //| TIM_CCMR1_IC1F_2; //
 TIM2->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP | TIM_CCER_CC1NE); //rising edge
 TIM2->CCMR1 &= ~(TIM_CCMR1_IC1PSC); //input prescaler
 TIM2->CCER |= TIM_CCER_CC1E; //enable capture from counter
 TIM2->DIER |= TIM_DIER_CC1IE; //enable interrupt request
 //TIM2->ARR = ARR_MAX /*ARR_COUNT*/;
 // enabling interrupt for capture/compare1 and update (overflow?)
 //TIM2->DIER = TIM_DIER_CC1IE | TIM_DIER_UIE;
 // set timer2 control register to enable timer
 //TIM2->CR1 = TIM_CR1_CEN;
 // set capture control to 200
 //TIM2->CCR1 = 399;
 // enable capture/control
 //TIM2->CCER = TIM_CCER_CC1E;
 TIM2->CR1 |= 1;
}
char *freq_to_string(uint32_t num, char* string, uint32_t str_len) {
	int32_t i = 0;
	string[str_len-1] = '\0';
	while(i < str_len -2 && num > 0) {
		string[str_len-i-3] = (num % 10) + '0'; //convert int value to numbered string
		num /= 10;
		i++;
	}
	string[str_len-1] = string[str_len-2];
	string[str_len-2] = string[str_len-3];
	string[str_len-3] = string[str_len-4];
	string[str_len-4] = '.';
	return &(string[str_len-i-2]);
}
void ConfigureGPIO(void) {
 // enable clock
 RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
 // set output
 //GPIOA->MODER &= ~(GPIO_MODER_MODE5_Msk | GPIO_MODER_MODE6_Msk);
 //GPIOA->MODER |= (1 << GPIO_MODER_MODE5_Pos | 1 << GPIO_MODER_MODE6_Pos);
 //GPIOA->OTYPER &= ~(GPIO_OTYPER_OT5_Msk | GPIO_OTYPER_OT6_Msk);
 //GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED5_Msk | GPIO_OSPEEDR_OSPEED6_Msk);
 //GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD5_Msk | GPIO_PUPDR_PUPD6_Msk);
 GPIOA->MODER &= ~(GPIO_MODER_MODE5_Msk);
 GPIOA->MODER |= (2 << GPIO_MODER_MODE5_Pos);
 GPIOA->OTYPER &= ~(GPIO_OTYPER_OT5_Msk);
 GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED5_Msk);
 GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD5_Msk);
 GPIOA->AFR[0] |= (1 << 20);
}
void ConfigureInterrupts(void) {
 // hook up interrupt
 NVIC->ISER[0] = 1 << (TIM2_IRQn & 0x1F);
 // enable
 __enable_irq();
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
/**
* @brief USART2 Initialization Function
* @param None
* @retval None
*/
//static void MX_USART2_UART_Init(void)
//{
///* USER CODE BEGIN USART2_Init 0 */
///* USER CODE END USART2_Init 0 */
///* USER CODE BEGIN USART2_Init 1 */
///* USER CODE END USART2_Init 1 */
//huart2.Instance = USART2;
//huart2.Init.BaudRate = 115200;
//huart2.Init.WordLength = UART_WORDLENGTH_8B;
//huart2.Init.StopBits = UART_STOPBITS_1;
//huart2.Init.Parity = UART_PARITY_NONE;
//huart2.Init.Mode = UART_MODE_TX_RX;
//huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//huart2.Init.OverSampling = UART_OVERSAMPLING_16;
//huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
//huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
//if (HAL_UART_Init(&huart2) != HAL_OK)
//{
//  Error_Handler();
//}
/* USER CODE BEGIN USART2_Init 2 */
/* USER CODE END USART2_Init 2 */
//}
/**
* @brief GPIO Initialization Function
* @param None
* @retval None
*/
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */
/* GPIO Ports Clock Enable */
__HAL_RCC_GPIOA_CLK_ENABLE();
/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
