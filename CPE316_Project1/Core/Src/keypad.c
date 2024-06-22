/*
 * keypad.c
 *
 *  Created on: May 7, 2024
 *      Author: vicer
 */


#include "keypad.h"
#include "main.h"


/* Initialize Keypad Peripheral (Using Port C) */
void keypad_Setup(void) {
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

	/* Pins 0-3 are Input, Pins 9,10,11,12 are Output */
	  GPIOC->MODER &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1 |	// Set MODE to Input (00) for Pins 0-3
			  	  	  	GPIO_MODER_MODE2 | GPIO_MODER_MODE3 |
						GPIO_MODER_MODE9 | GPIO_MODER_MODE10 |	// Bit Mask Pins 9,10,11,12
						GPIO_MODER_MODE11 | GPIO_MODER_MODE12);
	  GPIOC->MODER |= (GPIO_MODER_MODE9_0 | GPIO_MODER_MODE10_0 |	// Set MODE to Output (01) for Pins 9,10,11,12
			  	  	   GPIO_MODER_MODE11_0 | GPIO_MODER_MODE12_0);

	  GPIOC->OTYPER &= ~(GPIO_OTYPER_OT9 | GPIO_OTYPER_OT10 |	// Set OTYPE to PP (0)
						 GPIO_OTYPER_OT11 | GPIO_OTYPER_OT12);

	  GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED9 | GPIO_OSPEEDR_OSPEED10 |	// Set OSPEED to Low-Speed (00)
			  	  	  	  GPIO_OSPEEDR_OSPEED11 | GPIO_OSPEEDR_OSPEED12);

	  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD0_Msk | GPIO_PUPDR_PUPD1_Msk |	// Set PUPD to PD (10)
			  	  	    GPIO_PUPDR_PUPD2_Msk | GPIO_PUPDR_PUPD3_Msk);
	  GPIOC->PUPDR |= (GPIO_PUPDR_PUPD0_1 | GPIO_PUPDR_PUPD1_1 |
			  	  	   GPIO_PUPDR_PUPD2_1 | GPIO_PUPDR_PUPD3_1);
}


/* Set all columns to 1 (Reset/Polling State) */
void keypad_Reset(void) {
	GPIOC->ODR = (GPIO_ODR_OD9 | GPIO_ODR_OD10 | GPIO_ODR_OD11 | GPIO_ODR_OD12);	// Set all Columns to 1
}


/* Read keypad value */
int8_t keypad_Read(void) {
	uint8_t rowBits = 0x0;
	uint8_t row;
	keypad_Reset();

	if ((GPIOC->IDR & 0xF) != 0x0) { // if first four pins are not 0
			 for (uint8_t column = 0; column < 4; column++) {
				 /* Set ODR to the current output bit */
				 switch(column) {
				 	 case 0: GPIOC->ODR = GPIO_ODR_OD9;
				 	 	 	 break;
				 	 case 1: GPIOC->ODR = GPIO_ODR_OD10;
				 	 	 	 break;
				 	 case 2: GPIOC->ODR = GPIO_ODR_OD11;
				 	 	 	 break;
				 	 case 3: GPIOC->ODR = GPIO_ODR_OD12;
				 	 	 	 break;
				 	 default: GPIOC->ODR = 0x0;
				 }

				 for(uint8_t i = 0; i < 5; i++); // Delay

				 rowBits = (GPIOC->IDR & 0xF);
				 //if ((rowCnt > (0x0 + (column*4))) && (rowCnt <= ((column + 1)*4))) {
				 if (rowBits != 0) {
					 switch(rowBits) {
						 case 0x1: row = 0;
								   break;
						 case 0x2: row = 1;
								   break;
						 case 0x4: row = 2;
								   break;
						 case 0x8: row = 3;
								   break;
						 default: row = 0;
					 	 	      break;
					 }

					 return btns[row][column];
				 }
			 }
	} else { return NO_BUTTON_PRESS;}

	return NO_BUTTON_PRESS;
}
