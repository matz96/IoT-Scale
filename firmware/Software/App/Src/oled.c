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

/*
 * Init for display
 */
void oled_init() {
	OLEDSemaphore = xSemaphoreCreateMutex();
	xSemaphoreGive(OLEDSemaphore);
	HAL_GPIO_WritePin(GPIOB, OLED_RST_PIN, GPIO_PIN_SET); // Set LCD-Reset-Pin
	if (xSemaphoreTake(I2CSemaphore,DELAY_MAX_I2C_SEMAPHORE) == pdTRUE) {
		ssd1306_Init();
		ssd1306_SetDisplayOn(OLED_ON);
		ssd1306_Fill(White);
		xSemaphoreGive(I2CSemaphore);
	}
}

/*
 * Print one line of text on the display
 */
int oled_printf(uint32_t line, SSD1306_COLOR color, const char *fmt, ...) {
	char s[22];
	va_list ap;
	if (xSemaphoreTake(OLEDSemaphore,DELAY_MAX_I2C_SEMAPHORE) == pdTRUE) {
		va_start(ap, fmt);
		vsnprintf(s, sizeof(s), fmt, ap);
		va_end(ap);
		if (xSemaphoreTake(I2CSemaphore,DELAY_MAX_I2C_SEMAPHORE) == pdTRUE) {
			ssd1306_SetCursor(OLED_BORDER_OFFSET,
					(line - 1) * OLED_LINE_HEIGHT + OLED_BORDER_OFFSET);
			ssd1306_WriteString(s, Font_6x8, color);
			ssd1306_UpdateScreen();
			xSemaphoreGive(I2CSemaphore);
		}
		xSemaphoreGive(OLEDSemaphore);
	}
	return 0;
}

/*
 * Print weight with the legend of the buttons on the display
 */
int oled_print_weight(SSD1306_COLOR color,const char *fmt, ...) {
	char s[22];
	va_list ap;
	if (xSemaphoreTake(OLEDSemaphore,DELAY_MAX_I2C_SEMAPHORE) == pdTRUE) {
		va_start(ap, fmt);
		vsnprintf(s, sizeof(s), fmt, ap);
		va_end(ap);
		if (xSemaphoreTake(I2CSemaphore,DELAY_MAX_I2C_SEMAPHORE) == pdTRUE) {
			//Legend of the buttons
			ssd1306_SetCursor(LEGEND_X_OFFSET, LEGEND_Y_BOTTOM_LINE);
			ssd1306_WriteString("Tara",  Font_6x8, color);
			ssd1306_SetCursor(LEGEND_X_OFFSET, LEGEND_Y_TOP_LINE);
			ssd1306_WriteString("g/oz",  Font_6x8, color);

			//Print out weight
			ssd1306_SetCursor(OLED_BORDER_OFFSET, OLED_WEIGHT_LINE_POSITION);
			ssd1306_WriteString(s,  Font_16x26, color);

			//Update screen with the new values
			ssd1306_UpdateScreen();
			xSemaphoreGive(I2CSemaphore);
		}
		xSemaphoreGive(OLEDSemaphore);
	}
	return 0;
}

