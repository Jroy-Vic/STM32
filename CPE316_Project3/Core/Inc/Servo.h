/*
 * Servo.h
 *
 *  Created on: Jun 5, 2024
 *      Author: vicer
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "main.h"
#include "TIM.h"

/* MACROS */
#define DEF_SERVO_ARR	0x2710	// ('d10000) Creates 50Hz PWM Signal
#define DEF_SERVO_CCRX	0x00FA	// ('d250) Sets X Position to 0 Degrees
#define DEF_SERVO_CCRY	0x00FA 	// ('d250) Sets Y Position to 0 Degrees
#define DEF_SERVO_UPDATE 0x00FF	// ('d100) Toggles PWM Signal Periodically
#define SERVO_DELAY 0x1388		// ('d5000) Activates Servo for a Calibrated Duration
#define SERVO_POS_HANDLER 0x384	// ('d900) Handles Quick Jumps in Position
#define SERVO_POS_DELAY	0x2		// Delay for Position Handler
#define MAX_SERVO_POS	0x04E2	// ('d1250) 180 Degrees
#define MIN_SERVO_POS	0x00FA	// ('d250) 0 Degrees

/* Initialize ServoX and ServoY */
void SERVO_init(void);

/* Update ServoX and ServoY Position */
void SERVO_update(int16_t x, int16_t y);

/* Handle PWM Signal */
void SERVO_PWM_toggle(void);

/* Activate Servo */
void SERVO_activate(void);

#endif /* INC_SERVO_H_ */
