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
static void MainTask2(void *pvParameters);
static void ReglerISTTask(void *pvParameters);

static void user_pwm_setvalue(float value);
static void calc_weight(uint16_t current, uint16_t temp);
static uint32_t readCurrent(void);
static uint32_t readTemp(void);
static uint32_t current_average(uint32_t current);

extern ADC_HandleTypeDef hadc;

S_piregler piregler;
SemaphoreHandle_t ReglerSemaphore;
SemaphoreHandle_t DisplaySemaphore;
SemaphoreHandle_t I2CSemaphore;
SemaphoreHandle_t RPSemaphore;

bool unit_oz = false;
uint32_t weight = 0;
uint32_t tara = 0;
int32_t pwm = 0; //remove after testing
int32_t dist = 0; //remove after testing

void app_main(void) {	I2CSemaphore = xSemaphoreCreateMutex();
	ReglerSemaphore = xSemaphoreCreateBinary();
	DisplaySemaphore = xSemaphoreCreateBinary();
	RPSemaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(ReglerSemaphore);
	xSemaphoreGive(I2CSemaphore);
	xSemaphoreGive(RPSemaphore);
	//xTaskCreate(I2C2Task, "I2C2-Task", (configMINIMAL_STACK_SIZE + 80), NULL, (tskIDLE_PRIORITY + 1), NULL);
	xTaskCreate(MainTask, "Main-Task", (configMINIMAL_STACK_SIZE + 80), NULL,(tskIDLE_PRIORITY + 1), NULL);
	xTaskCreate(ReglerISTTask, "Regler-Task", (configMINIMAL_STACK_SIZE + 80),
	NULL, (tskIDLE_PRIORITY + 2), NULL);
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
 *
 */
static void MainTask(__attribute__ ((unused)) void *pvParameters) {
	HAL_Delay(DISPLAY_START_DELAY);
	oled_init();
	char text[DISPLAY_TEXT_LENGTH];
	uint32_t current = 0;
	uint32_t temp = 0;
	while (1) {
		if (xSemaphoreTake(DisplaySemaphore,
				MAX_DELAY_DISPLAY_SEMAPHORE) == pdTRUE) {
			if (unit_oz) {
				static uint32_t ounce = 0;
				ounce = (float) (weight - tara) * CONVERSION_GR_OZ;
				snprintf(text, sizeof(text), "Gew.: %3d oz", (int) ounce);
			} else {
				snprintf(text, sizeof(text), "Gew.: %3d g",
						(int) (weight - tara));
			}
			oled_printf(1, Black, text);
			snprintf(text, sizeof(text), "weight:");
			oled_printf(2, Black, text);
			snprintf(text, sizeof(text), "%06d", (int) weight);
			oled_printf(3, Black, text);
			snprintf(text, sizeof(text), "Current:");
			oled_printf(4, Black, text);
			snprintf(text, sizeof(text), "%06d", (int) current);
			oled_printf(5, Black, text);
			snprintf(text, sizeof(text), "PWM-Value:");
			oled_printf(6, Black, text);
			snprintf(text, sizeof(text), "%06d", (int) pwm);
			oled_printf(7, Black, text);
			snprintf(text, sizeof(text), "Dist:");
			oled_printf(8, Black, text);
			snprintf(text, sizeof(text), "%06d", (int) dist);
			oled_printf(9, Black, text);
			current = current_average(readCurrent());
			temp = readTemp();
			calc_weight(current, temp);
			HAL_Delay(500);
		}
	}
}

/*
 * MainTask for Display weight and read out current and temperature
 */
static void MainTask2(__attribute__ ((unused)) void *pvParameters) {
	HAL_Delay(DISPLAY_START_DELAY);
	oled_init();
	char text[DISPLAY_TEXT_LENGTH];
	uint32_t current = 0;
	uint32_t temp = 0;
	while (1) {
		if (xSemaphoreTake(DisplaySemaphore,
				MAX_DELAY_DISPLAY_SEMAPHORE) == pdTRUE) {
			if (unit_oz) {
				static int32_t ounce = 0;
				ounce = ((int32_t) weight - tara) * CONVERSION_GR_OZ;
				snprintf(text, sizeof(text), "%0.3doz", (int) ounce);
			} else {
				snprintf(text, sizeof(text), "%0.4dg", (int) (weight - tara));
			}
			oled_printf(1, Black, text);
			current = readCurrent();
			temp = readTemp();
			calc_weight(current, temp);
			xSemaphoreGive(RPSemaphore);
			HAL_Delay(DISPLAY_REFRESH_TIMEOUT);
		}
	}
}

/**
 * Task for communication over I2C with Raspberry Pi Zero W (over second i2c interface)
 */
static void I2C2Task(__attribute__ ((unused)) void *pvParameters) {
	//Write to RP over I2C2
	extern I2C_HandleTypeDef hi2c2;
	uint8_t cnt = 0;
	//snprintf(text, sizeof(text), "Test: %d", cnt++);
	while (1) {
		if (xSemaphoreTake(RPSemaphore,
				MAX_DELAY_RP_SEMAPHORE) == pdTRUE) {
			HAL_I2C_Master_Transmit(&hi2c2, RPZERO_ADDR, &cnt, 1,
					HAL_MAX_DELAY);
			cnt++;
		}
	}
}

/*
 * Interupt service task for PI-Controller
 */
static void ReglerISTTask(__attribute__ ((unused)) void *pvParameters) {
	initVCNL4040(VCNL4040_ADDR); //Wegmesssensor initialisieren
	int32_t idle_value = (DISTANCE_SCALER*readVCNL4040(VCNL4040_ADDR, (VCNL4040_PS_DATA)))
			- START_DIST_OFFSET; //Read start distance
	piregler_init(&piregler, idle_value, 0, KP, 0, KI, LOW, HIGH, TS); //PIRegler initialisieren
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

	pwm = pwmvalue; // Remove in final version

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
	current = (COVERTION_A_TO_MA * (float) current) / ADC_MAX_VALUE;
	current = current ^ 0b11;
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
	temp = temp ^ 0b11;
	//temp = (COVERTION_A_TO_MA * (float) temp) / ADC_MAX_VALUE;
	return (temp);
}

/**
 * Calculation weight
 */
static calc_weight(uint16_t current, uint16_t temp) {
	static const float a0 = 3;
	static const float a1 = 0.259086;
	static const float a2 = 0.00249554;
	static const float a3 = -0.000001443;
	weight = a0+(current*a1)+(current*current)*a2+(current*current*current)*a3;
}

static uint32_t current_average(uint32_t current){
	static uint32_t n = 0;
	static const uint16_t num_sample = 3;
	static uint32_t sample_current[3] = {0,0,0};
	uint32_t avg_current = 0;
	sample_current[n] = current;
	if(++n == 3){
		n = 0;
	}
	for(int i = 0; i < num_sample; i++){
		avg_current += sample_current[i];
	}
	return(avg_current/num_sample);
}
