/*
 * i2c1.c
 *
 *  Created on: Nov 30, 2021
 *      Author: Daniel Moser
 */

#include "i2c1.h"

void i2c_bus_init(void) {

}

void i2c_set_sda(void) {

}

void i2c_clear_sda(void) {

}

uint8_t i2c_get_sda(void) {

	return(0);
}

void i2c_set_scl(void) {

}

void i2c_clear_scl(void) {

}

/**
 * @brief I2C master write 8-bit data bit-bang
 * @param unsigned char b - data to transmit
 * @retval unsigned char ack – acknowledgement received
 */
void I2C_master_write(unsigned char b) {
	unsigned char msk = 0x80;
	unsigned char ack;
	do {
		sda_wr_control(b);
		setup_dly();
		i2c_set_scl();
		half_bit_dly();
		i2c_clear_scl();
		setup_dly();
	} while ((msk >>= 1) != 0);
	i2c_set_sda();/* ACK slot checking */
	i2c_set_scl();
	half_bit_dly();
	ack = i2c_get_sda();
	i2c_clear_scl();
	return (ack);
}

/**
 * @brief I2C master read 8-bit bit-bang
 * @param unsigned char ack – acknowledgement control
 * @retval unsigned char b – data received
 */
unsigned char I2C_master_read(unsigned char ack) {
	unsigned char msk = 0x80;
	unsigned char b = 0;
	do {
		i2c_set_scl();
		half_bit_dly();
		sda_rd_control(b);
		i2c_clear_scl();
		half_bit_dly();
	} while ((msk >>= 1) != 0);
	if (ack != 0) {
		i2c_clear_sda();/* ACK slot control */
	}
	setup_dly();
	i2c_set_scl();
	half_bit_dly();
	i2c_clear_scl();
	half_bit_dly();
	return (b);
}
/**
 * @brief I2C start
 * @param none
 * @retval none
 */
void I2C_Start_condition(void) {
	i2c_bus_init();
	half_bit_dly();
	i2c_clear_sda();
	half_bit_dly();
	i2c_clear_scl();
	half_bit_dly();
}
/**
 * @brief I2C stop
 * @param none
 * @retval none
 */
void I2C_Stop_condition(void) {
	i2c_clear_sda();
	i2c_clear_scl();
	half_bit_dly();
	i2c_set_scl();
	half_bit_dly();
	i2c_set_sda();
	half_bit_dly();
}

