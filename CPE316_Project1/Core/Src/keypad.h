/*
 * keypad.h
 *
 *  Created on: May 7, 2024
 *      Author: vicer
 */

#ifndef SRC_KEYPAD_H_
#define SRC_KEYPAD_H_
#include "main.h"

#define NUM_ROWS 4
#define NUM_COLS 4
#define KEYPAD_A 10
#define KEYPAD_B 11
#define KEYPAD_C 12
#define KEYPAD_D 15
#define KEYPAD_STAR 13
#define KEYPAD_POUND 14
#define NO_BUTTON_PRESS -1

static const uint8_t btns[NUM_ROWS][NUM_COLS] =
	{{1, 2, 3, KEYPAD_A},
	{4, 5, 6, KEYPAD_B},
	{7, 8, 9, KEYPAD_C},
	{KEYPAD_STAR, 0, KEYPAD_POUND, KEYPAD_D}};


/* Initialize Keypad Peripheral */
void keypad_Setup(void);
/* Poll columns for key presses */
void keypad_Reset(void);
/* Read keypad value */
int8_t keypad_Read(void);


#endif /* SRC_KEYPAD_H_ */
