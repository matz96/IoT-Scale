/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
//#include "vcnl4040.h"
#include "ssd1306.h"
#include "ssd1306_tests.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


#define LOWER true
#define UPPER false

#define VCNL4040_ALS_CONF 0x00
#define VCNL4040_ALS_THDH 0x01
#define VCNL4040_ALS_THDL 0x02
#define VCNL4040_PS_CONF1 0x03 //Lower
#define VCNL4040_PS_CONF2 0x03 //Upper
#define VCNL4040_PS_CONF3 0x04 //Lower
#define VCNL4040_PS_MS 0x04 //Upper
#define VCNL4040_PS_CANC 0x05
#define VCNL4040_PS_THDL 0x06
#define VCNL4040_PS_THDH 0x07
#define VCNL4040_PS_DATA 0x08
#define VCNL4040_ALS_DATA 0x09
#define VCNL4040_WHITE_DATA 0x0A
#define VCNL4040_INT_FLAG 0x0B //Upper
#define VCNL4040_ID 0x0C

#define OLED_BORDER_OFFSET 5
#define OLED_LINE_HEIGHT 10

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
const uint8_t VCNL4040_ADDR = 0x60<<1; //7-bit unshifted I2C address of VCNL4040
const uint8_t RPZERO_ADDR = 0x03<<1;
HAL_StatusTypeDef send_command_vcnl4040(uint8_t address, uint8_t command, uint16_t data);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
void init_vcnl4040(void);
void receive_data_vcnl4040(void);
void user_pwm_setvalue(uint16_t value);
bool writeCommand(uint8_t commandCode, uint16_t value);
bool writeCommands(uint8_t commandCode, uint8_t lowbyte, uint8_t highbyte);
int32_t readSensor(uint8_t commandCode);
void oled_print(char text[], uint32_t line, SSD1306_COLOR color);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  //init_vcnl4040();
  uint16_t pwm_value = 0;
  uint16_t step = 0;
  uint16_t prox = 0;
  int32_t CH1_DC = 0;
  char text[20];
  HAL_StatusTypeDef ret;
  uint8_t buf[12];

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);

  ssd1306_Init();
  ssd1306_SetDisplayOn(1);
  ssd1306_Fill(White);
  CH1_DC = 10;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /*setLEDCurrent(50); //Max IR LED current
	  	setIRDutyCycle(40); //Set to highest duty cycle
	  	setProxIntegrationTime(8); //Set to max integration
	  	setProxResolution(16); //Set to 16-bit output
	  	enableSmartPersistance(); //Turn on smart presistance
	  	powerOnProximity(); //Turn on prox sensing*/



	  /*buf[0] = 0x08;
	  // Init register of VCNL4040
		writeCommands(0x04, 0b00010010, 0b00000111); // PS_CONF3_L & PS_MS
		writeCommands(0x03, 0b11001110, 0b00001000); // PS_CONF1_L & PS_CONF2_H
	 // writeCommands(0x04, 0b00001000, 0b00000111); // PS_CONF3_L & PS_MS
	 // writeCommands(0x03, 0b11001111, 0b00001000); // PS_CONF1_L & PS_CONF2_H

	  //writeCommands(0x06, 0x00, 0x00); //PS Threashold


	  while(1){
		  static uint16_t data;
		  static uint16_t interupt;
		  static uint16_t info;
		  static uint16_t als;

		  // write and read back
		  /*while(1){
			  static uint16_t test = 0;
			  writeCommands(0x03, 0b11001110, 0b00001011);
			  test = readSensor(0x03);
		  }*/

		  //read back config registers
	  /*
		  data  = readSensor(0x08);
		  //als = readSensor(0x09);
		  //interupt = readSensor(0x0B);
		  info = readSensor(0x0C);
		  HAL_Delay(2000);
	  }

	  //ret = HAL_I2C_Master_Transmit(&hi2c1, VCNL4040_ADDR, buf[0], 1, HAL_MAX_DELAY);

	  /*while(1)
	  {
		  ret = HAL_I2C_Master_Receive(&hi2c1, VCNL4040_ADDR, buf, 2, HAL_MAX_DELAY);
		  if(ret != HAL_OK){

		  }else{
		  }

		  //send_command_vcnl4040(0x0C);
		 // HAL_I2C_Master_Receive(&hi2c1, VCNL4040_ADDR, data, sizeof(uint8_t), HAL_MAX_DELAY);
		  HAL_Delay(2000);
	  }*/


	  /*
	   * Testfunction for I2C-2 with Raspberry Pi Zero W
	   */
	  //uint8_t cnt = 0;
	  //snprintf(text, sizeof(text), "Test: %d", cnt++);

	 /* while(1){
		  ret = HAL_I2C_Master_Transmit(&hi2c2, RPZERO_ADDR, &cnt,1, HAL_MAX_DELAY);
		  HAL_Delay(200);
		  cnt++;
	  }*/



	  /*
	   * PWM
	   */
	  //HAL_Delay(100);
	  //if(pwm_value == 0) step = 100;
	  //if(pwm_value == 2000) step = -100;
	  //pwm_value += step;
	  //user_pwm_setvalue(pwm_value);


	  //Prescale of PWM: 1000 -> ~8kHz
	  /*while (CH1_DC < 2000)
		{
			TIM1->CCR4 = CH1_DC;
			CH1_DC += 10;
			HAL_Delay(100);
		}
		while(CH1_DC > 0)
		{
			TIM1->CCR4 = CH1_DC;
			CH1_DC -= 10;
			HAL_Delay(100);
		}*/
	  /*/
	   * OLED
	   */
	  static int n = 0;
	  snprintf(text, sizeof(text), "Test: %d", n++);
	  oled_print(text, 1, Black);

/*
	   * VCNL4040
	   */
	  //get Proximity
	  /*
	  prox = getProximity();
	  snprintf(text, sizeof(text), "Dist: %d", prox);
	  oled_print(text, 2, Black);
	  */



	  HAL_Delay(2000);


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x1050DDFF;
  hi2c1.Init.OwnAddress1 = 192;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_DISABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x2010091A;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void init_vcnl4040(void){
	//if (getID() != 0x0186) return (false); //Check default ID value
	setLEDCurrent(50); //Max IR LED current
	setIRDutyCycle(40); //Set to highest duty cycle
	setProxIntegrationTime(8); //Set to max integration
	setProxResolution(16); //Set to 16-bit output
	enableSmartPersistance(); //Turn on smart presistance
	powerOnProximity(); //Turn on prox sensing

	 //send_command_vcnl4040(VCNL4040_ADDR,VCNL4040_PS_CONF2,(0b00001011)<<8); //enable 16-bit PS
	 //Default -> more to add for specific funcions
}

HAL_StatusTypeDef send_command_vcnl4040(uint8_t address, uint8_t command, uint16_t data){
	static uint8_t buf[4];
	buf[0] = address&0x01;
	buf[1] = command;
	buf[2] = data;
	return(HAL_I2C_Master_Transmit(&hi2c1, address, buf ,4, HAL_MAX_DELAY));
}


int32_t readSensor(uint8_t commandCode)
{

	uint16_t MemoryAdresse = (commandCode<<8) + (VCNL4040_ADDR+1);
	uint8_t databuf[2] = {0x00, 0x00};

	HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(&hi2c1, VCNL4040_ADDR, MemoryAdresse, 2, databuf, 2, HAL_MAX_DELAY);
	//HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(&hi2c1, VCNL4040_ADDR, data, 2, HAL_MAX_DELAY);
	//HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(&hi2c1, VCNL4040_ADDR, commandCode, 2, data, 2, HAL_MAX_DELAY);
	//HAL_I2C_Mem_Write(&hi2c1, VCNL4040_ADDR, commandCode, uint, pData, Size, Timeout)

   if (ret != HAL_OK) //Send a restart command. Do not release bus.
    {
      return (-1); //Sensor did not ACK
    }
    //ret = HAL_I2C_Master_Receive(&hi2c1, VCNL4040_ADDR, data, 2, HAL_MAX_DELAY);
    if(ret == HAL_OK)
    {
    	return((databuf[1]<<8) + databuf[0]);
    }
     return (-1); //Sensor did not respond
}

bool writeCommand(uint8_t commandCode, uint16_t value)
{
	uint8_t	LSB =  (value & 0xFF);
	uint8_t MSB = (value >> 8);
	return (writeCommands(commandCode, LSB, MSB));
}

bool writeCommands(uint8_t commandCode, uint8_t lowbyte, uint8_t highbyte)
{
	static HAL_StatusTypeDef ret;
	//uint8_t	LSB =  lowbyte;
	//uint8_t MSB = highbyte;
	static uint8_t data[3];
	data[0] = commandCode;
	data[1] = lowbyte;
	data[2] = highbyte;

	//HAL_I2C_MAster
   ret = HAL_I2C_Master_Transmit(&hi2c1, VCNL4040_ADDR, data, 3, HAL_MAX_DELAY);
   //ret = HAL_I2C_Master_Transmit(&hi2c1, VCNL4040_ADDR, data, 2, HAL_MAX_DELAY);
   //ret = HAL_I2C_Master_Transmit(&hi2c1, VCNL4040_ADDR, &MSB, 1, HAL_MAX_DELAY);
   //HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(&hi2c1, VCNL4040_ADDR, &commandCode, 1, HAL_MAX_DELAY);
   if(ret != HAL_OK){
	   return (false); //Sensor did not ACK
   }
   return true;
}




void read_data_vcnl4040(uint8_t address, uint8_t command){
	//static uint8_t buf[3];
	//buf[0] = address&0x01;
	//buf[1] = command;
	//buf[2] = address&0x01;
	//HAL_I2C_Master_Transmit(&hi2c1, VCNL4040_ADDR, pData, 1, HAL_MAX_DELAY)
}


void user_pwm_setvalue(uint16_t value)
{

}

void oled_print(char text[], uint32_t line, SSD1306_COLOR color){
	  ssd1306_SetCursor(OLED_BORDER_OFFSET, (line-1)*OLED_LINE_HEIGHT+OLED_BORDER_OFFSET);
	  ssd1306_WriteString(text,  Font_6x8, color);
	  ssd1306_UpdateScreen();
}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
