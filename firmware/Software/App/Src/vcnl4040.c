/*
 * vcnl4040.c
 *
 *  Created on: Nov 30, 2021
 *      Author: Daniel Moser
 */

#include "vcnl4040.h"

/*	// Init register of VCNL4040
	writeCommands(0x00, 0b11000001, 0b00000000); //ALS Conf
	writeCommands(0x03, 0b11111110, 0b00001000); // PS_CONF1_L & PS_CONF2_H
	writeCommands(0x04, 0b01101101, 0b10100111); // PS_CONF3_L & PS_MS
	writeCommands(0x06, 0x00, 0x00); //PS Threashold
*/


bool writeCommands(uint8_t commandCode, uint8_t lowbyte, uint8_t highbyte)
{
	/*
	static HAL_StatusTypeDef ret;
	//uint8_t	LSB =  lowbyte;
	//uint8_t MSB = highbyte;
	static uint8_t data[3];
	data[0] = commandCode;
	data[1] = lowbyte;
	data[2] = highbyte;

   ret = HAL_I2C_Master_Transmit(&hi2c1, VCNL4040_ADDR, data, 3, HAL_MAX_DELAY);
   //ret = HAL_I2C_Master_Transmit(&hi2c1, VCNL4040_ADDR, data, 2, HAL_MAX_DELAY);
   //ret = HAL_I2C_Master_Transmit(&hi2c1, VCNL4040_ADDR, &MSB, 1, HAL_MAX_DELAY);
   //HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(&hi2c1, VCNL4040_ADDR, &commandCode, 1, HAL_MAX_DELAY);
   if(ret != HAL_OK){
	   return (false); //Sensor did not ACK
   }*/
   return true;
}


int32_t readSensor(uint8_t commandCode)
{
	/*
	uint8_t data[2]={commandCode, VCNL4040_ADDR + 1};

	//HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(&hi2c1, VCNL4040_ADDR, data, 2, HAL_MAX_DELAY);
	HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(&hi2c1, VCNL4040_ADDR, commandCode, 2, data, 2, HAL_MAX_DELAY);
	//HAL_I2C_Mem_Write(&hi2c1, VCNL4040_ADDR, commandCode, uint, pData, Size, Timeout)

   if (ret != HAL_OK) //Send a restart command. Do not release bus.
    {
      return (0); //Sensor did not ACK
    }
    ret = HAL_I2C_Master_Receive(&hi2c1, VCNL4040_ADDR, data, 2, HAL_MAX_DELAY);
    if(ret == HAL_OK)
    {
    	return((data[1]<<8) + data[0]);
    }*/
     return (-1); //Sensor did not respond
}
