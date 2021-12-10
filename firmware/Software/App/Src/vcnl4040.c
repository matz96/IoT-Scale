/*
 * vcnl4040.c
 *
 *  Created on: Nov 30, 2021
 *      Author: Daniel Moser
 */

#include "vcnl4040.h"
#include <stdio.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f0xx_hal.h"
#include "semphr.h"

extern I2C_HandleTypeDef hi2c1;
extern SemaphoreHandle_t I2CSemaphore;

void initVCNL4040(uint8_t addr){

	writeVCNL4040(addr,0x04, 0b00010011, 0b00000111); // PS_CONF3_L & PS_MS
	writeVCNL4040(addr,0x03, 0b11001110, 0b00001000); // PS_CONF1_L & PS_CONF2_H
}

bool writeVCNL4040(uint8_t addr, uint8_t command, uint8_t lowbyte, uint8_t highbyte){
	static HAL_StatusTypeDef ret;
	static uint8_t data[3];
	data[0] = command;
	data[1] = lowbyte;
	data[2] = highbyte;
	if(xSemaphoreTake(I2CSemaphore,1000) == pdTRUE){
		ret = HAL_I2C_Master_Transmit(&hi2c1, addr, data, 3, HAL_MAX_DELAY);
		xSemaphoreGive(I2CSemaphore);
	}
    if(ret != HAL_OK){
	   return (false); //Sensor did not ACK
    }
    return true;
}

int32_t readVCNL4040(uint8_t addr, uint8_t command){
	HAL_StatusTypeDef ret;
	uint16_t MemoryAdresse = (command<<8) + (addr+1);
	uint8_t databuf[2] = {0x00, 0x00};
	if(xSemaphoreTake(I2CSemaphore,1000) == pdTRUE){
		ret = HAL_I2C_Mem_Read(&hi2c1, addr, MemoryAdresse, 2, databuf, 2, HAL_MAX_DELAY);
		xSemaphoreGive(I2CSemaphore);
	}
    if (ret != HAL_OK) //Send a restart command. Do not release bus.
    {
      return (-1); //Sensor did not ACK
    }
    if(ret == HAL_OK)
    {
    	return((databuf[1]<<8) + databuf[0]);
    }
     return (-1); //Sensor did not respond
}



