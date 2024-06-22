/*
 * EEPROM.c
 *
 *  Created on: Jun 2, 2024
 *      Author: vicer
 */

#include "EEPROM.h"
#include "main.h"

/* Intialize EEPROM Peripheral Using I2C Communication */
/* PB8 = SCL
 * PB9 = SDA
 */
void EEPROM_init() {
	/* Configure GPIO Pins for I2C1 */
	/* Enable Clocks for GPIOB and I2C1 */
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
	RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;
	/* Set MODER to Alt. Func (10) */
	GPIOB->MODER &= ~(GPIO_MODER_MODE8 | GPIO_MODER_MODE9);
	GPIOB->MODER |= (GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1);
	/* Set OTYPER to Open Drain (1) */
	GPIOB->OTYPER |= (GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9);
	/* Set OSPEEDR to Very Fast (11) */
	GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEED8 | GPIO_OSPEEDR_OSPEED9);
	/* Set PUPDR to PU (01) */
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD8 | GPIO_PUPDR_PUPD9);
	GPIOB->PUPDR |= (GPIO_PUPDR_PUPD8 | GPIO_PUPDR_PUPD9);
	/* Set AFRH to AF4 (0100) */
	GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL8 | GPIO_AFRH_AFSEL9);
	GPIOB->AFR[1] |= (GPIO_AFRH_AFSEL8_2 | GPIO_AFRH_AFSEL9_2);

	/* Configure I2C1 */
	/* Clear PE Bit in I2C1_CR1 */
	I2C1->CR1 &= ~I2C_CR1_PE;
	/* Configure I2C1_TIMINGR */
	I2C1->TIMINGR = TIMING_FACTOR;
	/* Enable NOSTRETCH in I2C_CR1 */
	I2C1->CR1 &= ~I2C_CR1_NOSTRETCH;
	/* Enable Analog and Digital Noise Filter to Create Noise Immunity
	 * Analog Filter: ANFOFF is Cleared
	 * Digital Filter: DNF is set to Four I2C Clock Cycles */
	I2C1->CR1 &= ~I2C_CR1_ANFOFF;
	I2C1->CR1 |= (DNF_CNT << I2C_CR1_DNF_Pos);
	/* Enable I2C1 by Setting PE Bit */
	I2C1->CR1 |= I2C_CR1_PE;
}


/* Configure Transmission/Reception Settings */
void EEPROM_bus_config(uint8_t rd_wrn, uint8_t data_size) {
	/* Reset State of I2C1->CR2 */
	I2C1->CR2 = CLEAR;

	/* Configure Reception Settings:
	 * ADD10: 7-Bit Address (0)
	 * SADD[7:1] Contains Bus Address
	 * RD_WRN: Write Transfer (0) / Read Transfer (1)
	 * NBYTES: Send a Byte of Data
	 * AUTOEND: Manually Send STOP Bit (0) */
	I2C1->CR2 &= ~I2C_CR2_ADD10;
	I2C1->CR2 |= (EEPROM_ADDR << (I2C_CR2_SADD_Pos + 0x1));
	if (rd_wrn) {
		I2C1->CR2 |= I2C_CR2_RD_WRN;
	} else {
		I2C1->CR2 &= ~I2C_CR2_RD_WRN;
	}
	I2C1->CR2 |= (data_size << I2C_CR2_NBYTES_Pos);
	I2C1->CR2 &= ~I2C_CR2_AUTOEND;

	/* Ensure START Condition is Asserted:
	 * SDA and SCL are Idle High
	 * No Deadlock Caused by Unwanted Signals */
	while (!((GPIOB->IDR & GPIO_IDR_ID8) && (GPIOB->IDR & GPIO_IDR_ID9)) && data_size != READ_DATA_WIDTH);
}


/* Write Memory Address in Bus (For EEPROM_read()) */
void EEPROM_write_memaddr(uint16_t addr) {
	/* Configure Bus Settings to Write Memory Address */
	EEPROM_bus_config(WRITE_TRANSFER, MEM_DATA_WIDTH);

	/* Launch Communication */
	I2C1->CR2 |= I2C_CR2_START;

	/* Transmit Memory Address */
	for (uint8_t i = 0; i < MEM_DATA_WIDTH; i++) {
		/* Wait for ACK */
		while (!(I2C1->ISR & I2C_ISR_TXE));

		/* Transmit Splices */
		switch (i) {
		/* Memory Address High: Bits 14-8 */
		case 0:
			I2C1->TXDR = (addr >> 0x8);
			break;
		/* Memory Address Low: Bits 7-0 */
		case 1:
			I2C1->TXDR = (addr & 0x00FF);
			break;
		}
	}

	/* End Communication */
	I2C1->CR2 |= I2C_CR2_STOP;
}


/* Read Byte from Provided 15-Bit Address */
int8_t EEPROM_read(uint16_t addr) {
	/* Write Memory Address to Bus */
	EEPROM_write_memaddr(addr);

	/* Change Bus Config. to Read Transfer of One Data Byte + Control Byte */
	EEPROM_bus_config(READ_TRANSFER, READ_DATA_WIDTH);

	/* Re-launch Communication */
	I2C1->CR2 |= I2C_CR2_START;

	/* Wait for ACK */
	while (!(I2C1->ISR & I2C_ISR_RXNE));

	/* Read Data Byte */
	int8_t outputData = I2C1->RXDR;

	/* End Communication */
	I2C1->CR2 |= I2C_CR2_STOP;

	return outputData;
}


/* Write Byte to Provided 15-Bit Address */
void EEPROM_write(uint16_t addr, int8_t data) {
	/* Configure Bus Settings to Write Data to EEPROM */
	EEPROM_bus_config(WRITE_TRANSFER, WRITE_DATA_WIDTH);

	/* Launch Communication */
	I2C1->CR2 |= I2C_CR2_START;

	/* Transmit Memory Address and Data */
	for (uint8_t i = 0; i < WRITE_DATA_WIDTH; i++) {
		/* Wait for ACK */
		while (!(I2C1->ISR & I2C_ISR_TXE));

		/* Transmit Splices */
		switch (i) {
		/* Memory Address High: Bits 14-8 */
		case 0:
			I2C1->TXDR = (addr >> 0x8);
			break;
		/* Memory Address Low: Bits 7-0 */
		case 1:
			I2C1->TXDR = (addr & 0x00FF);
			break;
		/* Data Byte */
		case 2:
			I2C1->TXDR = data;
			break;
		}
	}

	/* End Communication */
	I2C1->CR2 |= I2C_CR2_STOP;

	/* Wait 5ms for Data to be Written to EEPROM */
	EEPROM_delay();
}


/* Wait 5ms for Data to be Written to EEPROM */
void EEPROM_delay(void) {
	for (uint16_t i = 0; i < FIVE_MS_DELAY; i++);
}
