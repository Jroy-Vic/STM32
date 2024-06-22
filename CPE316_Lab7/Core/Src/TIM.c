/*
 * TIM.c
 *
 *  Created on: May 22, 2024
 *      Author: vicer
 */

#include "main.h"
#include "TIM.h"


/* Initialize Timer (Set to 24MHz, Prescaled Down to 4MHz) */
void TIM_init() {
  /* Enabling clock for TIM2 (24MHz) */
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

  TIM2->CR1 &= ~TIM_CR1_CEN;	// Temporarily turn off Timer
  TIM2->PSC = TIM2_PRESCALER;	// Scale Clock down to 4MHz to Save Power
  TIM2->ARR = (uint16_t) ARR_VAL;
  //TIM2->CCR1 = (uint16_t) CCR_VAL;

  /* Set TIM2 interrupts to be highest priority */
  NVIC->IP[TIM2_IRQn] = TIM2_PRIORITY;
  /* Enable NVIC to handle TIM2 interrupts */
  NVIC->ISER[TIM2_NVIC] = (1 << (TIM2_IRQn & 0x1F));
  __enable_irq();

  /* Set Timer Conditions and Enable */
  TIM2->CR1 &= ~TIM_CR1_UDIS;	// Enable UEVs
  TIM2->DIER |= (TIM_DIER_UIE /*| TIM_DIER_CC1IE*/);	// Enable hardware interrupt
  TIM2->CR1 |= TIM_CR1_CEN;	// Enable timer
  TIM2->EGR |= TIM_EGR_UG;	// Force Update Event to reset timer
  TIM2->EGR |= ~TIM_EGR_UG;	// Toggle off Force Update Event
}


/* Force Timer Reset */
void TIM_reset() {
	TIM2->EGR |= TIM_EGR_UG;	// Force Update Event to reset timer
	TIM2->EGR |= ~TIM_EGR_UG;	// Toggle off Force Update Event
}


/* Change ARR Value */
void TIM_setARR(uint16_t arr) {
	/* Set ARR Value for TIM2 */
	TIM2->ARR = arr;

	/* Reset Timer */
	TIM_reset();
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
	/* If ARR is reached, Set Sample_Flag */
	Sample_Flag = SET;
}
