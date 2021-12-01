/*
 * i2c1.h
 *
 *  Created on: Nov 30, 2021
 *      Author: engel
 */

#ifndef INC_I2C1_H_
#define INC_I2C1_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "FreeRTOS.h"
#include "task.h"
//#include "queue.h"
#include "semphr.h"
#include "ssd1306.h"

void i2c_bus_init(void);
void i2c_set_sda(void);
void i2c_clear_sda(void);
uint8_t i2c_get_sda(void);
void i2c_set_scl(void);
void i2c_clear_scl(void);

void I2C_master_write(unsigned char);
unsigned char I2C_master_read(unsigned char);
void I2C_Start_condition(void);
void I2C_Stop_condition(void);



#endif /* INC_I2C1_H_ */
