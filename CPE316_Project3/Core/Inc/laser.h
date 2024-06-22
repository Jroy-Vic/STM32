/*
 * laser.h
 *
 *  Created on: Jun 7, 2024
 *      Author: vicer
 */

#ifndef INC_LASER_H_
#define INC_LASER_H_

#include "main.h"
#include "tim.h"
#include "Servo.h"

/* Initialize Laser Peripheral */
void LASER_init(void);

/* Activate Laser */
void LASER_activate(void);


#endif /* INC_LASER_H_ */
