/*
 * oled.h
 *
 *  Created on: Nov 30, 2021
 *      Author: Daniel Moser
 */

#ifndef OLED_H_
#define OLED_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "../ssd1306/ssd1306.h"

#define OLED_BORDER_OFFSET 15
#define OLED_LINE_HEIGHT 30
#define OLED_WEIGHT_LINE_POSITION 40
#define LEGEND_X_OFFSET 104
#define LEGEND_Y_BOTTOM_LINE 60
#define LEGEND_Y_TOP_LINE 10
#define DELAY_MAX_I2C_SEMAPHORE 1000
#define OLED_ON 1
#define OLED_OFF 0
#define OLED_RST_PIN GPIO_PIN_2

void oled_init();
int oled_printf (uint32_t line, SSD1306_COLOR color,const char *__restrict, ...);
int oled_print_weight(SSD1306_COLOR color,const char *__restrict, ...);

SemaphoreHandle_t OLEDSemaphore;

#endif /* OLED_H_ */
