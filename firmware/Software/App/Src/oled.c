/*
 * oled.c
 *
 *  Created on: Nov 30, 2021
 *      Author: Daniel Moser
 */

#include "oled.h"
#include "semphr.h"
#include "ssd1306.h"
#include "stdio.h"
#include "stdarg.h"

void oled_init(){
	ssd1306_Init();
	ssd1306_SetDisplayOn(1);
	ssd1306_Fill(White);
	OLEDSemaphore = xSemaphoreCreateBinary();
}

int oled_printf (uint32_t line, SSD1306_COLOR color,const char *fmt, ...){
	char s[SSD1306_WIDTH];
	va_list ap;
	if(xSemaphoreTake(OLEDSemaphore,10) == pdTRUE){
		va_start(ap, fmt);
		vsnprintf(s, sizeof(s), fmt, ap);
		va_end(ap);
		ssd1306_SetCursor(OLED_BORDER_OFFSET, (line-1)*OLED_LINE_HEIGHT+OLED_BORDER_OFFSET);
		ssd1306_WriteString(s,  Font_6x8, color);
		ssd1306_UpdateScreen();
		xSemaphoreGive(OLEDSemaphore);
	}
	return 0;
}

