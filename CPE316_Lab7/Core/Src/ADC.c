/*
 * ADC.c
 *
 *  Created on: May 16, 2024
 *      Author: vicer
 */

#include "main.h"
#include "ADC.h"

/* Initialize ADC1 Peripheral */
/* Running the ADC with a 24MHz Clock
 * Single Conversion, initiated with SC Bit
 * Using sampler; Hold timer with sample time of 640.5 clocks
 * 12-bit Conversion using 3.3V Reference
 * Configure analog input pin
 */
void ADC_init() {
	/* Configure Analog Input Pin for Channel 1 (PC0) */
	/* Enable GPIOC Clock */
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
	/* Set GPIO to Analog Mode for ADC (11) */
	GPIOC->MODER |= GPIO_MODER_MODE0;
	/* Connect Analog Switch to the ADC Input (1) */
	GPIOC->ASCR |= GPIO_ASCR_ASC0;

	/* Enable 24MHz ADC Clock and set to HCLK/1 (Synchronous Mode) */
	RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;
	ADC123_COMMON->CCR |= ADC_CCR_CKMODE_0;

	/* Power Up ADC (Turn off Deep Power Down Mode) */
	ADC1->CR &= ~ADC_CR_DEEPPWD;
	/* Enable Voltage Regulator */
	ADC1->CR |= ADC_CR_ADVREGEN;
	/* Wait 20 us to ensure regulator startup time has elapsed */
	for (uint8_t i = 0; i < TWENTYU_DELAY; i++);
	while (!(ADC1->CR & ADC_CR_ADVREGEN));

	/* Ensure ADC is Disabled*/
	ADC1->CR &= ~ADC_CR_ADEN;

	/* Select Input Mode for Calibration (Single-ended Input [0]) */
	ADC1->CR &= ~ADC_CR_ADCALDIF;
	/* Calibrate ADC and Wait Until Complete (ADCAL returns to 0) */
	ADC1->CR |= ADC_CR_ADCAL;
	while (ADC1->CR & ADC_CR_ADCAL);

	/* Set Channel 5 (PC0) as Single-ended Mode (0) */
	ADC1->DIFSEL &= ~ADC_DIFSEL_DIFSEL_1;

	/* Configure ADC1 (Clear ADSTART Initially) */
	ADC1->CR &= ~ADC_CR_ADSTART;
	/* Set to Single Conversion Mode (0) */
	ADC1->CFGR &= ~ADC_CFGR_CONT;
	/* Set to Right-Aligned Data (0) */
	ADC1->CFGR &= ~ADC_CFGR_ALIGN;
	/* Set to 12-bit Resolution (00) */
	ADC1->CFGR &= ~ADC_CFGR_RES;
	/* Set Channel 1 as a Single Regular Sequence (1) */
	ADC1->SQR1 |= ADC_SQR1_SQ1_0;
	/* Set Sample Time to 640.5 Clocks to Channel 1 (111) */
	ADC1->SMPR1 |= ADC_SMPR1_SMP1;
	/* Allow Conversions to be Set by Software (00) */
	ADC1->CFGR &= ~ADC_CFGR_EXTEN;

	/* Enable Interrupts at End of Conversions (EOC) */
	ADC1->IER |= ADC_IER_EOCIE;
	/* Enable Global Interrupt in NVIC */
	NVIC->ISER[ADC_NVIC] |= (SET << (ADC1_IRQn & 0x1F));
	NVIC->IP[ADC1_IRQn] = ADC_NVIC_PRIORITY;
	__enable_irq();

	/* Clear ADC Ready Flag (Write 1 to Bit), Then Enable ADC */
	ADC1->ISR |= ADC_ISR_ADRDY;
	ADC1->CR &= ~ADC_CR_ADDIS;
	ADC1->CR |= ADC_CR_ADEN;
	/* Hardware Sets ADRDY Flag; Wait for Bit to be Set */
	while(!(ADC1->ISR & ADC_ISR_ADRDY));
}


/* Begin a New Conversion
 * Sets ADSTART to Begin a New Conversion Sample
 * ADSTART is cleared by hardware when initiated
 */
void ADC_convert(void) {
	/* Begin ADC Conversion */
	ADC1->CR |= ADC_CR_ADSTART;
}


/* ADC Interrupt Handler */
/* Save Digital Conversion to a Global Variable
 * Set a Global Flag
 */
void ADC1_IRQHandler() {
	/* If Conversion has Ended, EOC Flag is Set */
	if (ADC1->ISR & ADC_ISR_EOC) {
		/* Save Digital Value to Global Variable */
		digital_Val = ADC1->DR;
		/* Reading from ADC1_DR Clears EOC Flag */

		/* Set Global Flag */
		ADC_Flag = SET;
	}
}


/* Converts 32-Bit Signed Value to String (Min. Value is 0) */
char* convInt_toStr(int32_t val) {
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
