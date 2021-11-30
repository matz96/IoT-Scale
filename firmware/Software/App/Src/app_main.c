/**
 * @file   app_main.c
 * @author Daniel Moser
 *
 */

#include "app_main.h"
#include <stdio.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f0xx_hal.h"
//#include "queue.h"
#include "semphr.h"
#include "ssd1306.h"
#include "oled.h"
#include "piregler.h"

#define PWM_MAX_VAL 2000
#define PWM_MIN_VAL 0
#define I2C_PORT GPIOB
#define I2C1_SDA GPIO_PIN_9
#define I2C1_SCL GPIO_PIN_8

static void I2CTask(void *pvParameters);
static void I2C2Task(void *pvParameters);
static void MainTask(void *pvParameters);
static void ReglerISTTask(void *pvParameters);

void user_pwm_setvalue(uint16_t value);
S_piregler piregler;



void app_main(void)
{
	xTaskCreate(I2CTask, "I2C-Task", (configMINIMAL_STACK_SIZE + 80), NULL, (tskIDLE_PRIORITY + 2), NULL);
	xTaskCreate(I2C2Task, "I2C2-Task", (configMINIMAL_STACK_SIZE + 80), NULL, (tskIDLE_PRIORITY + 1), NULL);
	xTaskCreate(MainTask, "Main-Task", (configMINIMAL_STACK_SIZE + 80), NULL, (tskIDLE_PRIORITY + 3), NULL);
	xTaskCreate(ReglerISTTask, "Regler-Task", (configMINIMAL_STACK_SIZE + 80), NULL, (tskIDLE_PRIORITY + 4), NULL);
	vTaskStartScheduler();
	/* The FreeRTOS scheduler should never return to here, except on out of memory at creating the idle task! */
	for (;;) ;
}

static void MainTask(__attribute__ ((unused)) void *pvParameters)
{
	static uint32_t n = 0;
	char text[22];
	while (1) {
		//try oled
		snprintf(text, sizeof(text), "Test: %d", (int)n++);
		oled_printf(*text, 1, Black);
		HAL_Delay(200);
		//Main Things
	}
}


static void I2CTask(__attribute__ ((unused)) void *pvParameters)
{
	//PB9 = SDA
	//PB8 = SCL
	HAL_GPIO_WritePin(I2C_PORT, I2C1_SCL, GPIO_PIN_SET);
	while (1) {

		//Do I2C Bit Banging
	}

}

static void I2C2Task(__attribute__ ((unused)) void *pvParameters)
{
	//Write to RP over I2C2
	extern I2C_HandleTypeDef hi2c2;
	const uint8_t RPZERO_ADDR = 0x03<<1;
	uint8_t cnt = 0;
	//snprintf(text, sizeof(text), "Test: %d", cnt++);
	while (1) {
		  HAL_I2C_Master_Transmit(&hi2c2, RPZERO_ADDR, &cnt,1, HAL_MAX_DELAY);
		  cnt++;
		  HAL_Delay(200);
	}

}

static void ReglerISTTask(__attribute__ ((unused)) void *pvParameters)
{
	while (1) {
		//Regeln
	}

}

void user_pwm_setvalue(uint16_t value)
{
	TIM1->CCR4 = value;
}

