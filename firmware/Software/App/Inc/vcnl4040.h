/*
 * vcnl4040.h
 *
 *  Created on: Nov 30, 2021
 *      Author: Daniel Moser
 */

#ifndef VCNL4040_H_
#define VCNL4040_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

//VCNL4040 Command Codes
#define VCNL4040_ALS_CONF 0x00
#define VCNL4040_ALS_THDH 0x01
#define VCNL4040_ALS_THDL 0x02
#define VCNL4040_PS_CONF1 0x03
#define VCNL4040_PS_CONF2 0x03
#define VCNL4040_PS_CONF3 0x04
#define VCNL4040_PS_MS 0x04
#define VCNL4040_PS_CANC 0x05
#define VCNL4040_PS_THDL 0x06
#define VCNL4040_PS_THDH 0x07
#define VCNL4040_PS_DATA 0x08
#define VCNL4040_ALS_DATA 0x09
#define VCNL4040_WHITE_DATA 0x0A
#define VCNL4040_INT_FLAG 0x0B
#define VCNL4040_ID 0x0C

void initVCNL4040(uint8_t addr);
bool writeVCNL4040(uint8_t addr, uint8_t command, uint8_t lowbyte, uint8_t highbyte);
int32_t readVCNL4040(uint8_t addr, uint8_t command);

//const uint8_t VCNL4040_ADDR = 0x60<<1; //7-bit unshifted I2C address of VCNL4040

#endif /* VCNL4040_H_ */
