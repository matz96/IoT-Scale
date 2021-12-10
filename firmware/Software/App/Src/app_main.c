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
#include "semphr.h"
#include "../ssd1306/ssd1306.h"
#include "oled.h"
#include "piregler.h"
#include "vcnl4040.h"

#define PWM_MAX_VAL 2000
#define PWM_MIN_VAL 0

#define IDLE_VALUE 16500
#define KP 5
#define TN 0.002 //2ms
#define LOW 14470
#define HIGH 1000
#define BIAS 0
#define TS 0.002

#define CONVERSION_GR_OZ  0.035274

#define BTN_TARA GPIO_PIN_10
#define BTN_UINT GPIO_PIN_11

static void I2CTask(void *pvParameters);
static void I2C2Task(void *pvParameters);
static void MainTask(void *pvParameters);
static void ReglerISTTask(void *pvParameters);

static void user_pwm_setvalue(float value);
static void calc_weight(uint16_t value);
static void EXTI2_3_IRQHandler(uint16_t GPIO_PIN);
static void readCurrent(void);

extern ADC_HandleTypeDef hadc;

S_piregler piregler;
SemaphoreHandle_t ReglerSemaphore;
SemaphoreHandle_t DisplaySemaphore;
SemaphoreHandle_t I2CSemaphore;
bool unit_oz = false;
uint32_t weight = 0;
uint32_t tara = 0;
int32_t dist = 0;
uint32_t current = 0;
int32_t pwm  = 0;


void app_main(void)
{
	I2CSemaphore = xSemaphoreCreateMutex();
	ReglerSemaphore = xSemaphoreCreateBinary();
	DisplaySemaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(ReglerSemaphore);
	xSemaphoreGive(I2CSemaphore);
	HAL_ADC_Init(&hadc);
	HAL_ADC_Start(&hadc);

	//xTaskCreate(I2CTask, "I2C-Task", (configMINIMAL_STACK_SIZE + 80), NULL, (tskIDLE_PRIORITY + 2), NULL);
	//xTaskCreate(I2C2Task, "I2C2-Task", (configMINIMAL_STACK_SIZE + 80), NULL, (tskIDLE_PRIORITY + 1), NULL);
	xTaskCreate(MainTask, "Main-Task", (configMINIMAL_STACK_SIZE + 80), NULL, (tskIDLE_PRIORITY + 2), NULL);
	xTaskCreate(ReglerISTTask, "Regler-Task", (configMINIMAL_STACK_SIZE + 80), NULL, (tskIDLE_PRIORITY + 2), NULL);
	vTaskStartScheduler();
	/* The FreeRTOS scheduler should never return to here, except on out of memory at creating the idle task! */
	for (;;) ;
}

/*
 * Timer for PI-Controller
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
	oled_init();
	char text[20];
	while (1) {
		if(xSemaphoreTake(DisplaySemaphore, 100) == pdTRUE){
			if(unit_oz){
				static uint32_t ounce = 0;
				ounce = (float)(weight-tara) * CONVERSION_GR_OZ;
				snprintf(text, sizeof(text), "Gew.: %3d oz", (int)ounce);
			}else{
				snprintf(text, sizeof(text), "Gew.: %3d gr", (int)(weight-tara));
			}
			oled_printf(1, Black, text);
			snprintf(text, sizeof(text), "Dist:");
			oled_printf(2, Black, text);
			snprintf(text, sizeof(text), "%06d", (int)dist);
			oled_printf(3, Black, text);
			snprintf(text, sizeof(text), "Current:");
			oled_printf(4, Black, text);
			snprintf(text, sizeof(text), "%06d", (int)current);
			oled_printf(5, Black, text);
			snprintf(text, sizeof(text), "PWM-Value:");
			oled_printf(6, Black, text);
			snprintf(text, sizeof(text), "%06d", (int)pwm);
			oled_printf(7, Black, text);
			HAL_Delay(200);
		}
	}

}
/**
 * Task can be deleted in final version. Only used for testing.
 */
static void I2CTask(__attribute__ ((unused)) void *pvParameters)
{
	static int32_t data = 0;
	initVCNL4040(0x60<<1);
	while (1) {
		// getID (0x0C): 0x0186 expected!! <- can be used for testing
		data = readVCNL4040(0x60<<1, 0x08);
		xSemaphoreGive(DisplaySemaphore);
		HAL_Delay(2000);
	}
}

/**
 * Task for communication over I2C with Raspberry Pi Zero W (over second i2c interface)
 */
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

/**
 * Interupt service task for PI-Controller
 */
static void ReglerISTTask(__attribute__ ((unused)) void *pvParameters)
{
	initVCNL4040(0x60<<1);
	piregler_init(&piregler, IDLE_VALUE, 0,KP , 0, TN, LOW, HIGH, BIAS, TS);
	static int32_t data = 0;
	while (1) {
		if(xSemaphoreTake(ReglerSemaphore,100) == pdTRUE){
			data = readVCNL4040(0x60<<1, (0x08));
			data  &= 0xFFFC; //Anti toggle
			dist = data;
			readCurrent();
			//dist = IDLE_VALUE-data; //inv
			piregler.val = (float)data;
			user_pwm_setvalue(ctl_pi(&piregler));
			//user_pwm_setvalue(dist);
			//user_pwm_setvalue(ctl_pi(&piregler)); //0 -2000

			xSemaphoreGive(DisplaySemaphore); //Call DisplayTask
		}
		HAL_Delay(20);
	}
}

/**
 * Set dutycyle of PWM-Output
 * Note: Value must be between PWM_MAX_VAL and PWM_MIN_VAL
 */
static void user_pwm_setvalue(float value)
{
	static int32_t pwmvalue = 1000;
	pwmvalue = pwmvalue - (value/1000);
	if(pwmvalue > PWM_MAX_VAL){
		pwmvalue = PWM_MAX_VAL;
	}
	if(pwmvalue <= PWM_MIN_VAL){
		pwmvalue = PWM_MIN_VAL;
	}

	pwm = pwmvalue;

	TIM1->CCR4 = pwmvalue;
	calc_weight(value);
}

/**
 * Read out ADC with the current value trough the FET
 */
static void readCurrent(void){
	current = HAL_ADC_GetValue(&hadc);
	current = (1000*(float)current) / 4096;
}

/**
 * Calculation weight
 */
static void calc_weight(uint16_t value){
	weight = value;
}

/**
 * Interupt-Function for Button-Action
 */
static void EXTI2_3_IRQHandler(uint16_t GPIO_PIN) {
	//Input Taster and set unit and tara
	if(GPIO_PIN == BTN_TARA){
		tara = weight;
	}
	if(GPIO_PIN == BTN_UINT){
		unit_oz =!unit_oz;
	}
}
