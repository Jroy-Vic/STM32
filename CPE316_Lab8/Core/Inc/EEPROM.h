/*
 * EEPROM.h
 *
 *  Created on: Jun 2, 2024
 *      Author: vicer
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#include "main.h"
/* MACROS */
#define CLEAR 0x0					// Resets Register
#define TIMING_FACTOR 0x00000E14	// Configures I2C_TIMINGR
#define DNF_CNT 0x1					// Configures Digital Noise Filter
#define EEPROM_ADDR 0x51			// I2C Bus Address for EEPROM
#define WRITE_DATA_WIDTH 0x4		/* Size of Data [Bytes] for Tx/Rx (Includes Memory Address)
									* Memory Address: 16 Bits
									* Data 8 Bit
									* Control Byte is included implicitly */
#define READ_DATA_WIDTH 0x2			// Only Includes Data Byte
#define MEM_DATA_WIDTH 0x3			// Only Includes Memory Address
#define FIVE_MS_DELAY 4000			// 5ms Delay to allow EEPROM to Write Data
#define READ_TRANSFER 0x1				// Used for Receiving
#define WRITE_TRANSFER 0x0			// Used for Transmitting

/* Intialize EEPROM Peripheral Using I2C Communication */
void EEPROM_init(void);

/* Configure Transmission/Reception Settings */
void EEPROM_bus_config(uint8_t rd_wrn, uint8_t data_size);

/* Write Memory Address in Bus (For EEPROM_read()) */
void EEPROM_write_memaddr(uint16_t addr);

/* Read Byte from Provided 15-Bit Address */
int8_t EEPROM_read(uint16_t addr);

/* Write Byte to Provided 15-Bit Address */
void EEPROM_write(uint16_t addr, int8_t data);

/* Wait 5ms for Data to be Written to EEPROM */
void EEPROM_delay(void);

#endif /* INC_EEPROM_H_ */
