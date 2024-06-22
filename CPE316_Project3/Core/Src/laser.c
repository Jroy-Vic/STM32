/*
 * laser.c
 *
 *  Created on: Jun 7, 2024
 *      Author: vicer
 */

#include "main.h"
#include "tim.h"
#include "Servo.h"


/* Initialize Laser Peripheral */
void LASER_init() {
	/* Using PA4 as Laser Output *
	 * AHB2ENR: GPIOA Clock Enable
	 * MODER = Output (01)
	 * OTYPER = PP (0)
	 * OSPEEDR = Low Speed (00)
	 * PUPD = PD (10)
	 */
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	GPIOA->MODER &= ~(GPIO_MODER_MODE4);
	GPIOA->MODER |= (GPIO_MODER_MODE4_0);
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT4);
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED4);
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD4);
	GPIOA->PUPDR |= (GPIO_PUPDR_PUPD4);
}


/* Activate Laser */
void LASER_activate() {
	/* Turn Laser on when Servo is Active */
	if ((TIM2->CCR1 > DEF_SERVO_CCRX) || (TIM2->CCR2 > DEF_SERVO_CCRY)) {
		GPIOA->ODR |= GPIO_ODR_OD4;
	} else {
		GPIOA->ODR &= ~GPIO_ODR_OD4;
	}
}
