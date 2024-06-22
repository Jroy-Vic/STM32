/*
 * tim2.c
 *
 *  Created on: Apr 26, 2024
 *      Author: vicer
 */

#include "main.h"
#include "tim2.h"


void TIM2_IRQHandler() {
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
