/*
 * TIM.h
 *
 *  Created on: Jun 5, 2024
 *      Author: vicer
 */

#ifndef INC_TIM_H_
#define INC_TIM_H_

#include "main.h"

/* MACROS */
#define TIM2_NVIC (((uint32_t)TIM2_IRQn) >> 5UL)	// Global Interrupt Enable
#define TIM2_PRIORITY 0x0				 // TIM2 Interrupt has Second Priority
#define TIM2_PRESCALER 0x7				 // Scale Clock Down from 4MHz to 500kHz

/* Global Variable */
extern uint8_t ServoX_Flag;
extern uint8_t ServoY_Flag;
extern uint8_t PWM_Flag;
extern uint8_t Update_Flag;

/* Initialize TIM2 Timer */
void TIM_init(uint16_t arr_val, uint16_t ccr1_val, uint16_t ccr2_val, uint16_t ccr3_val);

/* Force Timer Reset */
void TIM_reset(void);

/* Change CRR Values */
void TIM_setCCR(uint16_t ccr1, uint16_t ccr2);

/* Disable TIM2 */
void TIM_disable(void);

/* Re-Enable TIM2 */
void TIM_enable(void);


#endif /* INC_TIM_H_ */
