/*
 * TIM.h
 *
 *  Created on: May 22, 2024
 *      Author: vicer
 */

#ifndef SRC_TIM_H_
#define SRC_TIM_H_

#include "main.h"

/* MACROS */
#define ARR_VAL 0x0190
//#define CCR_VAL 0x00c8
#define TIM2_NVIC (((uint32_t)TIM2_IRQn) >> 5UL)
							// Global Interrupt Enable
#define TIM2_PRIORITY 0x1	// TIM2 Interrupt has Second Priority
#define TIM2_PRESCALER 0x5	// Scale Clock Down from 24MHz to 4MHz

/* Global Variable */
extern uint8_t Sample_Flag;


/* Initialize TIM2 Timer */
void TIM_init(void);

/* Force Timer Reset */
void TIM_reset(void);

/* Change ARR Value */
void TIM_setARR(uint16_t arr);

/* Disable TIM2 */
void TIM_disable(void);

/* Re-Enable TIM2 */
void TIM_enable(void);

/* TIM2 Interrupt Handler */
/* void TIM2_IRQHandler(void); */

#endif /* SRC_TIM_H_ */
