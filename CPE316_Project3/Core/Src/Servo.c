/*
 * Servo.c
 *
 *  Created on: Jun 5, 2024
 *      Author: vicer
 */

#include "main.h"
#include "Servo.h"
#include "TIM.h"


/* Initialize ServoX and ServoY */
void SERVO_init(void) {
	/* Initialize GPIO Output for PWM: PC0, PC1
	 * AHB2ENR: GPIOC Clock Enable
	 * MODER = Output (01)
	 * OTYPER = PP (0)
	 * OSPEEDR = High Speed (10)
	 * PUPD = PD (10)
	 */
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
	GPIOC->MODER &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1);
	GPIOC->MODER |= (GPIO_MODER_MODE0_0 | GPIO_MODER_MODE1_0);
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT0 | GPIO_OTYPER_OT1);
	GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED0 | GPIO_OSPEEDR_OSPEED1);
	GPIOC->OSPEEDR |= (GPIO_OSPEEDR_OSPEED0_1 | GPIO_OSPEEDR_OSPEED1_1);
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD0 | GPIO_PUPDR_PUPD1);
	GPIOC->PUPDR |= (GPIO_PUPDR_PUPD0_1 | GPIO_PUPDR_PUPD1_1);
	/* Set PCO and PC1 */
	GPIOC->ODR |= (GPIO_ODR_OD0 | GPIO_ODR_OD1);

	/* Initialize Timer to Create PWM */
	TIM_init(DEF_SERVO_ARR, DEF_SERVO_CCRX, DEF_SERVO_CCRY, DEF_SERVO_UPDATE);
}


/* Update ServoX and ServoY Position
 * If -1 is Input, Retain Previous value
 */
void SERVO_update(int16_t x, int16_t y) {
	/* Active when Update_Flag is Set */
	if (Update_Flag == SET) {
		/* Clear Update_Flag */
		Update_Flag = CLEARED;

		/* Retain Previous Value if -1 */
		if (x == NEGATIVE) {
			x = TIM2->CCR1;
		}
		if (y == NEGATIVE) {
			y = TIM2->CCR2;
		}

		/* Compare Difference in Previous to Current Values */
		int16_t dX = (TIM2->CCR1 - x);
		int16_t dY = (TIM2->CCR2 - y);
		if (dX < 0) {
			dX *= NEGATIVE;
		}
		if (dY < 0) {
			dY *= NEGATIVE;
		}

		/* Change CCR Values to Adjust Position */
		TIM_setCCR(x, y);

		/* Handle Quick Position Changes */
		if ((dX > SERVO_POS_HANDLER) || (dY > SERVO_POS_HANDLER)) {
			for (uint8_t i = 0; i < SERVO_POS_DELAY; i++) {
			  SERVO_activate();
			  SERVO_update(x,y);
		  }
		}
	}
}


/* Handle PWM Signal */
void SERVO_PWM_toggle() {
	/* Output HIGH Signal to GPIO when PWM_Flag is Set */
	if (PWM_Flag == SET) {
		GPIOC->ODR |= (GPIO_ODR_OD0 | GPIO_ODR_OD1);
		/* Clear Flag */
		PWM_Flag = CLEARED;
	}

	/* Pull Signal LOW if Servo_Flag is Set */
	if (ServoX_Flag == SET) {
		GPIOC->ODR &= ~GPIO_ODR_OD0;
		/* Clear Flag */
		ServoX_Flag = CLEARED;
	}
	if (ServoY_Flag == SET) {
		GPIOC->ODR &= ~GPIO_ODR_OD1;
		/* Clear Flag */
		ServoY_Flag = CLEARED;
	}
 }


/* Activate Servo */
void SERVO_activate() {
	for (uint32_t i = 0; i < SERVO_DELAY; i++) {
		SERVO_PWM_toggle();
	}
}
