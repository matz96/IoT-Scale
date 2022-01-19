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

static void MainTask(void *pvParameters);
static void ReglerISTTask(void *pvParameters);

static void user_pwm_setvalue(float value);
static void calc_weight(uint16_t current);
static uint32_t readCurrent(void);
static uint32_t readTemp(void);
static uint32_t current_average(uint32_t current);

extern ADC_HandleTypeDef hadc;

S_piregler piregler;
SemaphoreHandle_t ReglerSemaphore;
SemaphoreHandle_t DisplaySemaphore;
SemaphoreHandle_t I2CSemaphore;

bool unit_oz = false;
uint32_t tara = 0;
int32_t weight = 0;

void app_main(void) {	I2CSemaphore = xSemaphoreCreateMutex();
	ReglerSemaphore = xSemaphoreCreateBinary();
	DisplaySemaphore = xSemaphoreCreateBinary();
	RPSemaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(ReglerSemaphore);
	xSemaphoreGive(I2CSemaphore);
	xTaskCreate(MainTask, "Main-Task", (configMINIMAL_STACK_SIZE+20), NULL,(tskIDLE_PRIORITY + 2), NULL);
	xTaskCreate(ReglerISTTask, "Regler-Task", (configMINIMAL_STACK_SIZE), NULL, (tskIDLE_PRIORITY + 3), NULL);
	vTaskStartScheduler();
	/* The FreeRTOS scheduler should never return to here, except on out of memory at creating the idle task! */
	for (;;)
		;
}

/*
 * Timer for PI-Controller
 */
void TIM7_IRQHandler(void) {
	portBASE_TYPE higherPriorityTaskWoken = 0;
	if (ReglerSemaphore != NULL) {
		xSemaphoreGiveFromISR(ReglerSemaphore, &higherPriorityTaskWoken);
	}
	if (higherPriorityTaskWoken != 0) {
		taskYIELD();
	}
	//Clear TIM7 Update Event Flag
	TIM7->SR = ~TIM_IT_UPDATE;
}

/*
 * Read the button
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == BTN_TARA) {
		tara = weight;
	}
	if (GPIO_Pin == BTN_UINT) {
		unit_oz = !unit_oz;
	}
}

/*
 * MainTask for Display weight and read out current and temperature
 * and write data to raspberry pi
 */
static void MainTask(__attribute__ ((unused)) void *pvParameters) {
	HAL_Delay(DISPLAY_START_DELAY);
	oled_init();
	char text[DISPLAY_TEXT_LENGTH];
	uint32_t current = 0;
	uint32_t temp = 0;
	extern I2C_HandleTypeDef hi2c2;
	int32_t buf[RP_BUF_SIZE];
	while (1) {
		if (xSemaphoreTake(DisplaySemaphore,
				MAX_DELAY_DISPLAY_SEMAPHORE) == pdTRUE) {
			//Read values and calculate weight
			current = readCurrent();
			temp = readTemp();
			calc_weight(current_average(current));

			//warn user by overheat
			if(temp > MAX_TEMP_VALUE){
				snprintf(text, sizeof(text), "Overheat");
			}else{
				//Print weight in the selected unit
				if (unit_oz) {
					static float ounce = 0;
					ounce = ((float)weight - (float)tara) * CONVERSION_GR_OZ;
					snprintf(text, sizeof(text), "%.3doz ", (int)ounce);
				} else {
					snprintf(text, sizeof(text), "%.4dg ", (int)(weight - tara));
				}
				oled_print_weight(Black, text);
			}
		}

		// Generate data buffer for raspberry
		buf[0] = weight-tara;
		buf[1] = (int32_t)piregler.kp;
		buf[2] = (int32_t)piregler.ki;
		buf[3] = (int32_t)piregler.low;
		buf[4] = (int32_t)piregler.high;
		buf[5] = (int32_t)piregler.ts;
		buf[6] = (int32_t)piregler.idlevalue;

		// Write data buffer
		HAL_I2C_Master_Transmit(&hi2c2, RPZERO_ADDR, buf, sizeof(buf), HAL_MAX_DELAY);
		HAL_Delay(DISPLAY_REFRESH_TIMEOUT);
	}
}

/*
 * Interupt service task for PI-Controller
 */
static void ReglerISTTask(__attribute__ ((unused)) void *pvParameters) {
	int32_t dist = 0;
	initVCNL4040(VCNL4040_ADDR); //VCNL4040 initalise
	piregler_init(&piregler, IDLE_VALUE, 0, KP, 0, KI, LOW, HIGH, TS); //PIRegler initalise
	while (1) {
		if (xSemaphoreTake(ReglerSemaphore,MAX_DELAY_REGLER_SEMAPHORE) == pdTRUE) {
			dist = readVCNL4040(VCNL4040_ADDR, (VCNL4040_PS_DATA)); //Read distance of sensor
			piregler.val = (float) dist*DISTANCE_SCALER; //Set value to the piregler
			user_pwm_setvalue(ctl_pi(&piregler)); //Run piregler and set the new pwm duty-cycle
			xSemaphoreGive(DisplaySemaphore);
		}
	}
}

/**
 * Set dutycyle of PWM-Output
 * Note: Value must be between PWM_MAX_VAL and PWM_MIN_VAL
 */
static void user_pwm_setvalue(float value) {
	static int32_t pwmvalue = PWM_START_VALUE;
	pwmvalue = pwmvalue - (value / PWM_SCALER);

	//Limit output of PWM
	if (pwmvalue > PWM_MAX_VAL) {
		pwmvalue = PWM_MAX_VAL;
	}
	if (pwmvalue <= PWM_MIN_VAL) {
		pwmvalue = PWM_MIN_VAL;
	}
	TIM1->CCR4 = pwmvalue; //Set DutyCylcle in timer-register
}

/**
 * Read out ADC with the current value trough the FET
 */
static uint32_t readCurrent(void) {
	ADC_Select_CH0();
	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc, MAX_ADC_TIMEOUT);
	uint32_t current = HAL_ADC_GetValue(&hadc);
	HAL_ADC_Stop(&hadc);
	//Calculate the current trough the Magnet in [mA]
	current = current ^ 0b11; //suppress toggle
	current = (COVERTION_A_TO_MA * (float) current) / ADC_MAX_VALUE;
	return (current);
}

/**
 * Read out ADC with the current temperature from the magnet
 */
static uint32_t readTemp(void) {
	ADC_Select_CH3();
	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc, MAX_ADC_TIMEOUT);
	uint32_t temp = HAL_ADC_GetValue(&hadc);
	temp = HAL_ADC_GetValue(&hadc);
	HAL_ADC_Stop(&hadc);
	temp = temp ^ 0b11; //suppress toggle
	return (temp);
}

/**
 * Calculation weight
 */
static void calc_weight(uint16_t current) {
	//Gewicht
	static const float a1 = 0.328283;
	static const float a2 = 0.00115748;
	static const float a3 = -0.00000018519;
	weight = (current*a1)+((uint32_t)current*current)*a2+((uint32_t)current*current*current)*a3;

	//Korrekturfaktor
	static const float m = 1.05524;
	static const float b = -21.7452;
	weight = m*((float)(weight))+b;

	// Weight not allowed under zero
	if(weight < 0){
		weight = 0;
	}
}

/*
 * Function to calculate average of last three data point from the current
 */
static uint32_t current_average(uint32_t current){
	static uint32_t n = 0;
	static uint32_t sample_current[NUM_AVERAGE_SAMPLES];
	uint32_t avg_current = 0;
	sample_current[n] = current;
	if(++n == NUM_AVERAGE_SAMPLES){
		n = 0;
	}
	for(int i = 0; i < NUM_AVERAGE_SAMPLES; i++){
		avg_current += sample_current[i];
	}
	return(avg_current/NUM_AVERAGE_SAMPLES);
}
