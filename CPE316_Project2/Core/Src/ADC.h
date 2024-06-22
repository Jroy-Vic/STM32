/*
 * ADC.h
 *
 *  Created on: May 16, 2024
 *      Author: vicer
 */

#ifndef SRC_ADC_H_
#define SRC_ADC_H_
#include "main.h"

/* MACROS */
#define TWENTYU_DELAY 80
#define ADC_NVIC (((uint32_t)ADC1_IRQn) >> 5UL)
#define ADC_NVIC_PRIORITY 0x2	// ADC has Third Priority
#define SET 0x1
#define VOLT_STR_SIZE 5		// Size of String Sent to Terminal (Includes Decimal Point and '\0')
#define ZERO_CHAR '0'		// Zero Character
#define DECIMAL_CHAR '.'	// Decimal Character
#define DECIMAL_IDX 1		// Decimal Index in voltStr
#define NULL_BYTE '\0'		// Null Byte

/* MACRO Functions */
#define CALIBRATION(x) ((807 * (x)) - 8970)			// Experimental Calibration Equation [in uV]
#define MICRO_DIVIDE(x) (((x) == 0) ? 1000000 : \
						((x) == 1) ? 100000 : \
						((x) == 2) ? 10000 : 1) 	// Used to check for non-zero digits

/* Global Variables */
extern uint16_t digital_Val;
extern uint8_t ADC_Flag;

/* Initialize ADC Peripheral */
void ADC_init(void);

/* Begin a New Conversion */
void ADC_convert(void);

/* Converts 32-Bit Signed Value to String (Min. Value is 0) */
char* convInt_toStr(int32_t val);

#endif /* SRC_ADC_H_ */
