/*
 * TIM.c
 *
 *  Created on: Jun 5, 2024
 *      Author: vicer
 */

#include "TIM.h"
#include "main.h"

/* Extern Variables */
uint8_t ServoX_Flag;
uint8_t ServoY_Flag;
uint8_t PWM_Flag;
uint8_t Update_Flag;


/* Initialize Timer (Set to 4MHz, Prescaled Down to 500kHz) */
void TIM_init(uint16_t arr_val, uint16_t ccr1_val, uint16_t ccr2_val, uint16_t ccr3_val) {
  /* Enabling clock for TIM2 (4MHz) */
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

  TIM2->CR1 &= ~TIM_CR1_CEN;	// Temporarily turn off Timer
  TIM2->PSC = TIM2_PRESCALER;	// Scale Clock down to 500kHz to Save Power
  TIM2->ARR = arr_val;			// Creates 50Hz PWM
  TIM2->CCR1 = ccr1_val;		// ServoX Duration
  TIM2->CCR2 = ccr2_val;		// SErvoY Duration
  TIM2->CCR3 = ccr3_val;		// Toggle Timer

  /* Set TIM2 interrupts to be highest priority */
  NVIC->IP[TIM2_IRQn] = TIM2_PRIORITY;
  /* Enable NVIC to handle TIM2 interrupts */
  NVIC->ISER[TIM2_NVIC] = (1 << (TIM2_IRQn & 0x1F));
  __enable_irq();

  /* Set Timer Conditions and Enable */
  TIM2->CR1 &= ~TIM_CR1_UDIS;	// Enable UEVs
  TIM2->DIER |= (TIM_DIER_UIE | TIM_DIER_CC1IE
		  	  | TIM_DIER_CC2IE | TIM_DIER_CC3IE);	// Enable hardware interrupt
  TIM2->CR1 |= TIM_CR1_CEN;	// Enable timer
  TIM2->EGR |= TIM_EGR_UG;	// Force Update Event to reset timer
  TIM2->EGR |= ~TIM_EGR_UG;	// Toggle off Force Update Event
}


/* Force Timer Reset */
void TIM_reset() {
	TIM2->EGR |= TIM_EGR_UG;	// Force Update Event to reset timer
	TIM2->EGR |= ~TIM_EGR_UG;	// Toggle off Force Update Event

	/* Clear Flags */
	PWM_Flag = CLEARED;
	ServoX_Flag = CLEARED;
	ServoY_Flag = CLEARED;
	Update_Flag = CLEARED;
}


/* Change CRR Values */
void TIM_setCCR(uint16_t ccr1, uint16_t ccr2) {
	/* Set CCR1 and CCR2 Value for TIM2 */
	TIM2->CCR1 = ccr1;
	TIM2->CCR2 = ccr2;
}


/* Disable TIM2 */
void TIM_disable() {
	TIM2->CR1 &= ~TIM_CR1_CEN;	// Temporarily turn off Timer
}

/* Re-Enable TIM2 */
void TIM_enable() {
	TIM2->CR1 |= TIM_CR1_CEN;	// Enable timer
	TIM_reset();
}


/* TIM2 Interrupt Handler */
void TIM2_IRQHandler() {
	/* If CCR1 is reached, Set ServoX_Flag */
	if (TIM2->SR & TIM_SR_CC1IF) {
		ServoX_Flag = SET;
		/* Clear CCR1IF */
		TIM2->SR &= ~TIM_SR_CC1IF;
	}
	/* If CCR2 is reached, Set ServoY_Flag */
	if (TIM2->SR & TIM_SR_CC2IF) {
		ServoY_Flag = SET;
		/* Clear CCR2IF */
		TIM2->SR &= ~TIM_SR_CC2IF;
	}
	/* If ARR is reached, Set PWM_Flag */
	if (TIM2->SR & TIM_SR_UIF) {
		PWM_Flag = SET;
		/* Clear UIF */
		TIM2->SR &= ~TIM_SR_UIF;
	}
	/* If CCR3 is reached, Set Update_Flag */
	if (TIM2->SR & TIM_SR_CC3IF) {
		Update_Flag = SET;
		/* Clear UIF */
		TIM2->SR &= ~TIM_SR_CC3IF;
	}
}

