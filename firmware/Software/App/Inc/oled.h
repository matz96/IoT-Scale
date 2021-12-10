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
//#include "queue.h"
#include "semphr.h"
#include "../ssd1306/ssd1306.h"

#define OLED_BORDER_OFFSET 5
#define OLED_LINE_HEIGHT 10

void oled_init();
int oled_printf (uint32_t line, SSD1306_COLOR color,const char *__restrict, ...);

SemaphoreHandle_t OLEDSemaphore;


#endif /* OLED_H_ */
