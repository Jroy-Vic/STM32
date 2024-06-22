/*
 * ADC.h
 *
 *  Created on: May 16, 2024
 *      Author: vicer
 */

#ifndef SRC_ADC_H_
#define SRC_ADC_H_
#include "main.h"
#define TWENTYU_DELAY 80
#define ADC_NVIC 0
#define ADC_NVIC_PRIORITY 0
#define SET 0x1

/* Global Variables */
extern uint16_t digital_Val;
extern uint8_t ISR_Flag;

/* Initialize ADC Peripheral */
void ADC_init(void);

/* Begin a New Conversion */
void ADC_convert(void);

#endif /* SRC_ADC_H_ */
