/*
 * oled.c
 *
 *  Created on: Nov 30, 2021
 *      Author: Daniel Moser
 */

#include "oled.h"
#include "semphr.h"
#include "../ssd1306/ssd1306.h"
#include "stdio.h"
#include "stdarg.h"
#include "stm32f0xx_hal.h"
#include "semphr.h"

extern SemaphoreHandle_t I2CSemaphore;

#define OLED_RST_PIN GPIO_PIN_2

void oled_init(){
	OLEDSemaphore = xSemaphoreCreateMutex();
	xSemaphoreGive(OLEDSemaphore);

	HAL_GPIO_WritePin(GPIOB, OLED_RST_PIN, GPIO_PIN_SET);
	if(xSemaphoreTake(I2CSemaphore,1000) == pdTRUE){
		ssd1306_Init();
		ssd1306_SetDisplayOn(1);
		ssd1306_Fill(White);
		xSemaphoreGive(I2CSemaphore);
	}
}

int oled_printf (uint32_t line, SSD1306_COLOR color,const char *fmt, ...){
	char s[22];
	va_list ap;
	if(xSemaphoreTake(OLEDSemaphore,1000) == pdTRUE){
		va_start(ap, fmt);
		vsnprintf(s, sizeof(s), fmt, ap);
		va_end(ap);
		if(xSemaphoreTake(I2CSemaphore,1000) == pdTRUE){
			ssd1306_SetCursor(OLED_BORDER_OFFSET, (line-1)*OLED_LINE_HEIGHT+OLED_BORDER_OFFSET);
			ssd1306_WriteString(s,  Font_6x8, color);
			ssd1306_UpdateScreen();
			xSemaphoreGive(I2CSemaphore);
		}
		xSemaphoreGive(OLEDSemaphore);
	}
	return 0;
}
