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
#include "vcnl4040.h"

#define PWM_MAX_VAL 2000
#define PWM_MIN_VAL 0

#define IDLE_VALUE 500
#define KP 2
#define TN 0.001
#define LOW 35
#define HIGH 2350
#define BIAS 0
#define TS 0.001


static void I2CTask(void *pvParameters);
static void I2C2Task(void *pvParameters);
static void MainTask(void *pvParameters);
static void ReglerISTTask(void *pvParameters);

static void user_pwm_setvalue(uint16_t value);
static void calc_weight(uint16_t value);


S_piregler piregler;
SemaphoreHandle_t ReglerSemaphore;
bool unit_pd = false;
uint32_t weight = 0;
uint32_t dist = 0;


void app_main(void)
{
	OLEDSemaphore = xSemaphoreCreateBinary();
	ReglerSemaphore = xSemaphoreCreateBinary();
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
	initVCNL4040(0x60<<1);
	piregler_init(&piregler, IDLE_VALUE, 0,KP , 0, TN, LOW, HIGH, BIAS, TS);
	//xTaskCreate(I2CTask, "I2C-Task", (configMINIMAL_STACK_SIZE + 80), NULL, (tskIDLE_PRIORITY + 2), NULL);
	//xTaskCreate(I2C2Task, "I2C2-Task", (configMINIMAL_STACK_SIZE + 80), NULL, (tskIDLE_PRIORITY + 1), NULL);
	xTaskCreate(MainTask, "Main-Task", (configMINIMAL_STACK_SIZE + 80), NULL, (tskIDLE_PRIORITY + 3), NULL);
	xTaskCreate(ReglerISTTask, "Regler-Task", (configMINIMAL_STACK_SIZE + 80), NULL, (tskIDLE_PRIORITY + 4), NULL);
	vTaskStartScheduler();
	/* The FreeRTOS scheduler should never return to here, except on out of memory at creating the idle task! */
	for (;;) ;
}

/*
 * Timer for PIRegler
 */
void TIM7_IRQHandler(void){
	portBASE_TYPE higherPriorityTaskWoken = 0;
	if(ReglerSemaphore != NULL){
		xSemaphoreGiveFromISR(ReglerSemaphore, & higherPriorityTaskWoken);
	}
	if (higherPriorityTaskWoken != 0){
				taskYIELD();
	}
	//Clear TIM7 Update Event Flag
	TIM7->SR = ~TIM_IT_UPDATE;
}

static void MainTask(__attribute__ ((unused)) void *pvParameters)
{
	static uint32_t n = 0;
	char text[22];
	while (1) {


		snprintf(text, sizeof(text), "Gewicht: %d", (int)weight);


		//ssd1306_SetCursor(OLED_BORDER_OFFSET, (line-1)*OLED_LINE_HEIGHT+OLED_BORDER_OFFSET);
		ssd1306_Fill(White);
		ssd1306_WriteString(text,  Font_6x8, Black);
		ssd1306_UpdateScreen();


		//oled_printf(*text, 1, Black);
		//snprintf(text, sizeof(text), "Dist: %d", (int)dist);
		//oled_printf(*text, 1, Black);
		HAL_Delay(2000);

		// Print/calc weight and react on button input
	}
}


static void I2CTask(__attribute__ ((unused)) void *pvParameters)
{
	static int32_t data = 0;
	//initVCNL4040(0x60<<1);
	while (1) {
		// getID (0x0C): 0x0186 expected!!
		data = readVCNL4040(0x60<<1, 0x08);
		HAL_Delay(2000);
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
	static int32_t data = 0;
	while (1) {
		if(xSemaphoreTakeFromISR(ReglerSemaphore,100) == pdTRUE){
			data = readVCNL4040(0x60<<1, 0x08);
			dist = data;
			//dist = IDLE_VALUE-data; //inv
			piregler.val = (float)data;

			//user_pwm_setvalue(dist/10);
			//user_pwm_setvalue(dist);
			//user_pwm_setvalue(ctl_pi(&piregler)); //0 -2000
		}
		xSemaphoreGive(ReglerSemaphore);
	}
}

static void user_pwm_setvalue(uint16_t value)
{
	float res = 0;
	res = (float)value-LOW;
	res = res / (HIGH-LOW);
	res = res * PWM_MAX_VAL;
	value = (uint16_t)res;

	//value = (uint16_t)((float)value-LOW)/(HIGH-LOW)*PWM_MAX_VAL;
	TIM1->CCR4 = 1000;
	calc_weight(value);
}

static void calc_weight(uint16_t value){
	weight = value;
}

void EXTI2_3_IRQHandler(void) {
	//Input Taster and set uint and tara
}

