/*
 * USART.h
 *
 *  Created on: May 10, 2024
 *      Author: vicer
 */

#ifndef SRC_UART_H_
#define SRC_UART_H_

#include "main.h"
#define BAUD_RATE 0x0045
#define BITMASK 0xFF
#define ESC 0x1B
#define UART2_NVIC (((uint32_t)USART2_IRQn) >> 5UL)
#define UART2_PRIORITY 0x0	// UART has First Priority
#define INPUT_SIZE 2


/* Initialize and Configure USART2 */
void UART_init(void);

/* Send and print a string of characters to the Terminal */
void UART_print(char* inString);

/* Send and print a VT100 Escape Code to the Terminal */
void UART_ESC_Code(char* inCode);

/* Implement Delay */
void UART_delay(uint32_t time);


#endif /* SRC_UART_H_ */
