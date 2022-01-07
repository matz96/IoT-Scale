/**
 * @file   app_main.c
 * @author Daniel Moser
 *
 */

#include "app_main.h"
#include "main.h"
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


static void I2C2Task(void *pvParameters);
static void MainTask(void *pvParameters);
static void ReglerISTTask(void *pvParameters);

static void ADCTestTask(void *pvParameters);

static void user_pwm_setvalue(float value);
static void calc_weight(uint16_t value);
static uint32_t readCurrent(void);
static uint32_t readTemp(void);

extern ADC_HandleTypeDef hadc;

S_piregler piregler;
SemaphoreHandle_t ReglerSemaphore;
SemaphoreHandle_t DisplaySemaphore;
SemaphoreHandle_t I2CSemaphore;

bool unit_oz = false;
uint32_t weight = 0;
uint32_t tara = 0;
int32_t pwm  = 0; //remove after testing
int32_t dist = 0; //remove after testing

void app_main(void){
	I2CSemaphore = xSemaphoreCreateMutex();
	ReglerSemaphore = xSemaphoreCreateBinary();
	DisplaySemaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(ReglerSemaphore);
	xSemaphoreGive(I2CSemaphore);
	xTaskCreate(ADCTestTask, "ADC-Task", (configMINIMAL_STACK_SIZE + 80), NULL, (tskIDLE_PRIORITY + 1), NULL);
	//xTaskCreate(I2C2Task, "I2C2-Task", (configMINIMAL_STACK_SIZE + 80), NULL, (tskIDLE_PRIORITY + 1), NULL);
	//xTaskCreate(MainTask, "Main-Task", (configMINIMAL_STACK_SIZE + 80), NULL, (tskIDLE_PRIORITY + 1), NULL);
	//xTaskCreate(ReglerISTTask, "Regler-Task", (configMINIMAL_STACK_SIZE + 80), NULL, (tskIDLE_PRIORITY + 2), NULL);
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

/*
 * Read the button
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == BTN_TARA){
			tara = weight;
		}
	if(GPIO_Pin == BTN_UINT){
			unit_oz =!unit_oz;
	}
}

/**
 * Task for testing ADC
 */
static void ADCTestTask(__attribute__ ((unused)) void *pvParameters)
{

	uint32_t current = 0;
	uint32_t temp = 0;
	while (1) {
		current = readCurrent();
		temp = readTemp();
		HAL_Delay(200);
	}
}


/*
 *
 */
static void MainTask(__attribute__ ((unused)) void *pvParameters)
{
	HAL_Delay(200);
	oled_init();
	char text[20];
	uint32_t current = 0;
	uint32_t temp = 0;
	while (1) {
		if(xSemaphoreTake(DisplaySemaphore, 100) == pdTRUE){
			if(unit_oz){
				static uint32_t ounce = 0;
				ounce = (float)(weight-tara) * CONVERSION_GR_OZ;
				snprintf(text, sizeof(text), "Gew.: %3d oz", (int)ounce);
			}else{
				snprintf(text, sizeof(text), "Gew.: %3d g", (int)(weight-tara));
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
			snprintf(text, sizeof(text), "Temp:");
			oled_printf(8, Black, text);
			snprintf(text, sizeof(text), "%06d", (int)temp);
			oled_printf(9, Black, text);
			current = readCurrent();
			temp = readTemp();
			HAL_Delay(500);
		}
	}
}


/**
 * Task for communication over I2C with Raspberry Pi Zero W (over second i2c interface)
 */
static void I2C2Task(__attribute__ ((unused)) void *pvParameters)
{
	//Write to RP over I2C2
	extern I2C_HandleTypeDef hi2c2;
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
	int32_t idle_value = readVCNL4040(VCNL4040_ADDR, (VCNL4040_PS_DATA)) - START_DIST_OFFEST;
	piregler_init(&piregler, idle_value, 0,KP , 0, KI, LOW, HIGH, TS);
	while (1) {
		if(xSemaphoreTake(ReglerSemaphore,100) == pdTRUE){
			piregler.val = (float)dist;
			user_pwm_setvalue(ctl_pi(&piregler));
			dist = readVCNL4040(VCNL4040_ADDR, (VCNL4040_PS_DATA));
			xSemaphoreGive(DisplaySemaphore);
		}
	}
}


/**
 * Set dutycyle of PWM-Output
 * Note: Value must be between PWM_MAX_VAL and PWM_MIN_VAL
 */
static void user_pwm_setvalue(float value)
{
	static int32_t pwmvalue = PWM_MAX_VAL/2;
	pwmvalue = pwmvalue - (value/2000);
	if(pwmvalue > PWM_MAX_VAL){
		pwmvalue = PWM_MAX_VAL;
	}
	if(pwmvalue <= PWM_MIN_VAL){
		pwmvalue = PWM_MIN_VAL;
	}

	pwm = pwmvalue;

	TIM1->CCR4 = pwmvalue;
}

/**
 * Read out ADC with the current value trough the FET
 */
static uint32_t readCurrent(void){
	ADC_Select_CH0();
	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc, 1000);
	uint32_t current = HAL_ADC_GetValue(&hadc);
	HAL_ADC_Stop(&hadc);
	current = (1000*(float)current) / ADC_MAX_VALUE; // Current in mA
	calc_weight(current);
	return(current);
}


/**
 * Read out ADC with the current temperature from the magnet
 */
static uint32_t readTemp(void){
	ADC_Select_CH3();
	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc, 1000);
	uint32_t temp = HAL_ADC_GetValue(&hadc);
	temp = HAL_ADC_GetValue(&hadc);
	HAL_ADC_Stop(&hadc);
	temp = (1000*(float)temp) / ADC_MAX_VALUE;
	return(temp);
}


/**
 * Calculation weight
 */
static void calc_weight(uint16_t value){
	weight = value;
}
