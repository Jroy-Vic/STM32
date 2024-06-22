/*
 * UART.c
 *
 *  Created on: May 10, 2024
 *      Author: vicer
 */

#include "main.h"
#include "UART.h"
#include <stdlib.h>

/* Global Variable */
char inputArr[ARR_SIZE];
uint8_t arrIDX;
uint8_t Array_Flag;


/* Initialize and Configure USART2 */
void UART_init() {
	/* USART2 is being used since it is already
	 *  connected to the USB and Hardware Debugger */
	/* USART2 is AF7 for PA2-PA4:
	 * PA2 = USART2_TX
	 * PA3 = USART_RX
	 * PA4 = USART2_CK */

	/* GPIO Configuration */
	/* Set Clock */
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
	/* MODE = Alt. Func (10) */
	GPIOA->MODER &= ~(GPIO_MODER_MODE2 | GPIO_MODER_MODE3 |
					GPIO_MODER_MODE4);
	GPIOA->MODER |= (GPIO_MODER_MODE2_1 | GPIO_MODER_MODE3_1 |
					GPIO_MODER_MODE4_1);
	/* AFL = AF7 (0111) */
	GPIOA->AFR[0] |= (GPIO_AFRL_AFSEL2 | GPIO_AFRL_AFSEL3 |
					GPIO_AFRL_AFSEL4);
	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL2_3 | GPIO_AFRL_AFSEL3_3 |
					GPIO_AFRL_AFSEL4_3);
	/* OTYPER = PP (0) */
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT2 | GPIO_OTYPER_OT3 |
					GPIO_OTYPER_OT4);
	/* OSPEEDR = Very High Speed (11) */
	GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEED2 | GPIO_OSPEEDR_OSPEED3 |
					GPIO_OSPEEDR_OSPEED4);
	/* PUPDR = No PUPD (00) */
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD2 | GPIO_PUPDR_PUPD3 |
					GPIO_PUPDR_PUPD4);

	/* USART2 Configuration */
	/* Disable USART2 to configure (Clear UE Bit) */
	USART2->CR1 &= ~USART_CR1_UE;
	/* Set Word Length to 8 Bits (M0, M1 = 00) */
	USART2->CR1 &= ~USART_CR1_M1;
	USART2->CR1 &= ~USART_CR1_M0;
	/* Set Oversampling by 8 to account for Baud Rate
	 * Using 8 to save power */
	USART2->CR1 |= USART_CR1_OVER8;
	/* Configure STOP Bits (1 Stop Bit) */
	USART2->CR2 &= ~USART_CR2_STOP;

	/* Enable Interrupts */
	/* Enable Rx Not Empty Flag */
	USART2->CR1 |= USART_CR1_RXNEIE;
	/* Enable NVIC */
	NVIC->IP[USART2_IRQn] = UART2_PRIORITY;
	NVIC->ISER[UART2_NVIC] = (1<< (USART2_IRQn & 0x1F));

	/* Set Baud Rate */
	/* When OVER8 = 1: BRR[15:4] = USARTDIV[15:4]
	 * 				   BRR[3] must be kept cleared
	 * 				   BRR[2:0] = USARTDIV[3:0] shifted 1 bit to the right
	 */
	USART2->BRR = ((BAUD_RATE & 0xFFF0) | (BAUD_RATE & ~0x000F) |
					((BAUD_RATE & 0x000F) >> 1));

	/* Enable USART2 */
	USART2->CR1 |= (USART_CR1_TE | USART_CR1_RE |
					USART_CR1_UE);

}


/* Send and print a string of characters to the Terminal */
void UART_print(char* inString) {
	/* Transmit Entire String of Data */
	while (*inString != '\0') {
		/* Wait for Transmit Data Register to be Ready */
		while (!(USART2->ISR & USART_ISR_TXE));

		/* Transmit Individual Character (Only Bits 0 - 8)*/
		USART2->TDR = (*inString & BITMASK);
		inString++;
	}
}


/* Send and print a VT100 Escape Code to the Terminal */
void UART_ESC_Code(char* inCode) {
	/* Transmit Preceding ESC Code */
	/* Wait for Transmit Data Register to be Ready */
	while (!(USART2->ISR & USART_ISR_TXE));

	/* Transmit ESC Code (0x1B) (Only Bits 0 - 8) */
	USART2->TDR = (ESC & BITMASK);

	/* Transmit Entire String of Data */
	UART_print(inCode);
}


/* Implement Delay */
void UART_delay(uint32_t time) {
	for(uint32_t i = 0; i < time; i++);
}


/* USART2 Interrupt Handler */
void USART2_IRQHandler() {
	/* Check if Array is Full */
	if ((arrIDX > ARR_SIZE) || Array_Flag) {
		/* Clear Interrupt Flag */
		USART2->RQR |= USART_RQR_RXFRQ;

		/* Set Array_Flag */
		Array_Flag = SET;
		return;
	}

	/* Check if there is data being received */
	if (USART2->ISR & USART_ISR_RXNE) {
		/* Read Char and Clear Interrupt Flag */
		char receivedChar = USART2->RDR;

		/* If NULL_BYTE or NEW_LINE, Set Array_Flag */
		if ((receivedChar == NULL_BYTE) || (receivedChar == NEW_LINE) || (receivedChar == CARRIAGE)) {
			Array_Flag = SET;
		}

		/* Save Char to Array (if not a backspace) */
		if (!((receivedChar == BACKSPACE) || (receivedChar == DEL))) {
			inputArr[arrIDX++] = receivedChar;
		} else {
			arrIDX--;
		}

		/* Echo Received Data to Terminal */
		char inChar[2] = {receivedChar, '\0'};
		UART_print(inChar);
	}
}
