/*
 * dac.h
 *
 *  Created on: May 7, 2024
 *      Author: vicer
 */

#ifndef SRC_DAC_H_
#define SRC_DAC_H_

#include "main.h"
#include "keypad.h"

#define THREEV 3
#define DELAY 1000
#define INPUT_NUM 3
#define DIGIT1 100
#define DIGIT2 10
#define DIGIT3 1
#define BITMASK 0xFFF
#define DAC_CAL 0x3
#define DAC_SHIFT 0xC
#define MAXVOLT 330
#define MAXBIT 4095
#define HIGH_VOLT 300
#define LOW_VOLT 0

/* Initialize and Configure DAC Peripheral */
void DAC_init(void);

/* Map Digital Voltage Value to a range of 0 - 4095 for the DAC */
uint16_t DAC_Volt_Conv(uint16_t voltage_in);

/* Send Data to the DAC */
void DAC_Write(uint16_t LUT_volt);


#endif /* SRC_DAC_H_ */
