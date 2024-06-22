/*
 * USART.h
 *
 *  Created on: May 10, 2024
 *      Author: vicer
 */

#ifndef SRC_UART_H_
#define SRC_UART_H_

#include "main.h"

/* MACROS */
#define BAUD_RATE 0x0045							// ('d69) Sets Baud Rate
#define BITMASK 0x1FF								// Bitmask Used for Baud Rate
#define ESC 0x1B									// VT100 Code for "Esc"
#define UART2_NVIC (((uint32_t)USART2_IRQn) >> 5UL)	// Enable Global Interrupt
#define UART2_PRIORITY 0x1							// Give First Priority
#define NULL_BYTE	'\0'							// Null Byte for String
#define NEW_LINE	'\n'							// New Line Character
#define BACKSPACE	'\b'							// Backspace Character
#define DEL			0x7F							// ASCII Code for DEL
#define CARRIAGE	'\r'							// Carriage Return Character

/* Global Variables */
extern char inputArr[ARR_SIZE];
extern uint8_t arrIDX;
extern uint8_t Array_Flag;

/* Initialize and Configure USART2 */
void UART_init(void);

/* Send and print a string of characters to the Terminal */
void UART_print(char* inString);

/* Send and print a VT100 Escape Code to the Terminal */
void UART_ESC_Code(char* inCode);

/* Implement Delay */
void UART_delay(uint32_t time);


#endif /* SRC_UART_H_ */
