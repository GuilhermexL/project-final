/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include <math.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "bmp280.h"
#include "mpu6050.h"
#include "fonts.h"
#include "ssd1306.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRIG_PIN GPIO_PIN_6
#define TRIG_PORT GPIOA
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
BMP280_HandleTypedef bmp280;
uint8_t moveServo;
uint8_t open_drip = 0;
float capacidade = 80;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void HCSR04_Read (void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float altitudeBMP280 ,pressureBMP280, temperatureBMP280, humidityBMP280;

#define BMP280_ADDRESS 0x76
#define TSL2561_ADDR 0x39


MPU6050_t MPU6050;
BMP280_HandleTypedef bmp280;
uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;  // is the first value captured ?
uint8_t Distance  = 0;


uint16_t size;
char Data[50];
char Data1[20];
char Data2[20];

void atualizarDisplayMPU6050() {

	SSD1306_Clear(); //Seta todos os pixels do buffer para branco
  	//MPU
  	MPU6050_Read_All(&hi2c1, &MPU6050);

	sprintf(Data, "\nInclinacao:\n");
  	HAL_UART_Transmit(&huart2, Data, strlen(Data), 10000);

  	sprintf(Data, "Horizontal: %.2f\n", MPU6050.KalmanAngleX);
  	HAL_UART_Transmit(&huart2, Data, strlen(Data), 10000);

  	sprintf(Data, "Vertical: %.2f\n\n", MPU6050.KalmanAngleY);
  	HAL_UART_Transmit(&huart2, Data, strlen(Data), 10000);

  	SSD1306_GotoXY(0, 0); // Posiciona no início do display
  	sprintf(Data, "Inclinacao:");
  	SSD1306_Puts(Data, &Font_7x10, 1);

  	SSD1306_GotoXY(0, 16); // Posiciona no início do display
  	sprintf(Data, "Horizontal: %.2f", MPU6050.KalmanAngleX);
  	SSD1306_Puts(Data, &Font_7x10, 1);

  	// Exibir o ângulo Y no display
  	SSD1306_GotoXY(0, 28); // Posição na segunda linha (depende do tamanho da fonte)
  	sprintf(Data, "Vertical: %.2f", MPU6050.KalmanAngleY);
  	SSD1306_Puts(Data, &Font_7x10, 1);

  	// Atualizar o display para refletir as mudanças
  	SSD1306_UpdateScreen();

}

void atualizarDisplayBMP280() {



	while (!bmp280_read_float(&bmp280, &temperatureBMP280, &pressureBMP280, &humidityBMP280 , &altitudeBMP280)) {
			size = sprintf((char *)Data, "Temperature/pressure reading failed\n");
	}

	sprintf(Data,"\nClima: \n");
	HAL_UART_Transmit(&huart2, Data, strlen(Data), 10000);

    sprintf(Data,"Pressure: %0.1f\n",pressureBMP280);
    HAL_UART_Transmit(&huart2, Data, strlen(Data), 10000);

    sprintf(Data1,"Altitude: %0.1f\n",altitudeBMP280);
    HAL_UART_Transmit(&huart2, Data1, strlen(Data1), 10000);

    sprintf(Data2,"Temp: %0.2f\n\n",temperatureBMP280);
   	HAL_UART_Transmit(&huart2, Data2, strlen(Data2), 10000);

	SSD1306_Clear(); //Seta todos os pixels do buffer para branco

	// Exibir a pressão no display
	SSD1306_GotoXY(0, 0); // Posição na primeira linha
	sprintf(Data, "Clima: ");
	SSD1306_Puts(Data, &Font_7x10, 1);

	SSD1306_GotoXY(0, 16); // Posição na primeira linha
	sprintf(Data, "Pressure: %.1f", pressureBMP280);
	SSD1306_Puts(Data, &Font_7x10, 1);

	// Exibir a altitude no display
	SSD1306_GotoXY(0, 28); // Posição na segunda linha
	sprintf(Data1, "Altitude: %.1f", altitudeBMP280);
	SSD1306_Puts(Data1, &Font_7x10, 1);

	// Exibir a temperatura no display
	SSD1306_GotoXY(0, 42); // Posição na terceira linha
	sprintf(Data2, "Temp: %.2f", temperatureBMP280);
	SSD1306_Puts(Data2, &Font_7x10, 1);

	// Atualizar o display para refletir as mudanças
	SSD1306_UpdateScreen();

	HAL_Delay(1);

	//Buzzer
	while(temperatureBMP280 > 26.5){
		  int x;
		  for(x=100; x<200; x=x+10)
		  {
			 __HAL_TIM_SET_AUTORELOAD(&htim3, x*2);
			 __HAL_TIM_SET_COMPARE(&htim3  ,TIM_CHANNEL_3, x);
			 HAL_Delay(100);
		  }

		  while (!bmp280_read_float(&bmp280, &temperatureBMP280, &pressureBMP280, &humidityBMP280 , &altitudeBMP280)) {
		  			size = sprintf((char *)Data, "Temperature/pressure reading failed\n");
		  }

		  SSD1306_Clear();

		  SSD1306_GotoXY(0, 0); // Posição na primeira linha
		  sprintf(Data, "Aviso: ");
		  SSD1306_Puts(Data, &Font_7x10, 1);

		  SSD1306_GotoXY(0, 16); // Posição na primeira linha
		  sprintf(Data, "Temperatura Acima ");
		  SSD1306_Puts(Data, &Font_7x10, 1);

		  SSD1306_GotoXY(0, 28); // Posição na primeira linha
		  sprintf(Data, "do limite ! ");
		  SSD1306_Puts(Data, &Font_7x10, 1);

		  SSD1306_GotoXY(0, 42); // Posição na terceira linha
		  sprintf(Data2, "Temp: %.2f", temperatureBMP280);
		  SSD1306_Puts(Data2, &Font_7x10, 1);

		SSD1306_UpdateScreen();

		HAL_Delay(1500);
		//_________________________
	}
	__HAL_TIM_SET_COMPARE(&htim3  ,TIM_CHANNEL_3, 0);
}

void openDrip(){


	 SSD1306_Clear();


	 if(open_drip){
		 //Servo
		 for(int i = 400; i <= 2600; i++) {
			 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, i);
		 }
		 SSD1306_GotoXY(0, 16); // Posição na primeira linha
		 sprintf(Data, "Irrigacao Ativada!");
		 SSD1306_Puts(Data, &Font_7x10, 1);

		sprintf(Data,"\nIrrigacao Ativada!: \n");
		HAL_UART_Transmit(&huart2, Data, strlen(Data), 10000);

		 moveServo = ~moveServo;
		 open_drip = ~open_drip;
	 }else{
		 for(int i = 2600; i >= 400; i--) {
			 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, i);
		 }
		 SSD1306_GotoXY(0, 16); // Posição na primeira linha
		 sprintf(Data, "Irrigacao Desativada! ");
		 SSD1306_Puts(Data, &Font_7x10, 1);

		 sprintf(Data,"\nIrrigacao Desativada!: \n");
		 HAL_UART_Transmit(&huart2, Data, strlen(Data), 10000);

		 moveServo = ~moveServo;
		 open_drip = ~open_drip;

	 }

	 SSD1306_UpdateScreen();

}

void capacity(){

	SSD1306_Clear();

	//Ultra
	HCSR04_Read();
	HAL_Delay(1);


	 if (Distance > capacidade) {
		 SSD1306_GotoXY(0, 0); // Posição na primeira linha
		 sprintf(Data, "Alerta: ");
		 SSD1306_Puts(Data, &Font_7x10, 1);

		 SSD1306_GotoXY(0, 16); // Posição na primeira linha
		 sprintf(Data, "Estufa vazia !");
		 SSD1306_Puts(Data, &Font_7x10, 1);

		 sprintf(Data, "\nAlerta: \r\n");
		 HAL_UART_Transmit(&huart2, Data, strlen(Data), 10000);

		 sprintf(Data, "Estufa vazia ! \n");
		 HAL_UART_Transmit(&huart2, Data, strlen(Data), 10000);


	 }else{

		 float v = (1 - (Distance / capacidade)) * 100;
		 SSD1306_GotoXY(0, 0); // Posição na primeira linha
		 sprintf(Data, "Capacidade: ");
		 SSD1306_Puts(Data, &Font_7x10, 1);

		 SSD1306_GotoXY(0, 16); // Posição na primeira linha
		 sprintf(Data, "Estufa em : %.2f%%\n ", v);
		 SSD1306_Puts(Data, &Font_7x10, 1);

		 sprintf(Data, "\nCapacidade: \r\n");
		 HAL_UART_Transmit(&huart2, Data, strlen(Data), 10000);

		 sprintf(Data, "Estufa em : %.2f%%\n", v);
		 HAL_UART_Transmit(&huart2, Data, strlen(Data), 10000);

		 sprintf(Data, "Distancia: %.2f%%\n", Distance);
		 HAL_UART_Transmit(&huart2, Data, strlen(Data), 10000);

	 }

	 SSD1306_UpdateScreen();

}


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
	MX_USART2_UART_Init();
	MX_I2C1_Init();
	MX_TIM1_Init();
	MX_TIM14_Init();
	MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  	MPU6050_Init(&hi2c1);
  	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // Servo
	HAL_TIM_IC_Start_IT(&htim14, TIM_CHANNEL_1); // Ultra
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // Temperatura-Pressão-Altitude
	SSD1306_Init();
	uint8_t buffer[40] = {0};

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  	// SENSOR BMP280
	bmp280_init_default_params(&bmp280.params);
	bmp280.addr = BMP280_I2C_ADDRESS_0;
	bmp280.i2c = &hi2c1;

	while (!(bmp280_init(&bmp280, &bmp280.params)))
		{
			size = sprintf((char *)Data, "BMP280 initialization failed\n");
			HAL_UART_Transmit(&huart2, Data, strlen(Data), 10000);
			HAL_Delay(2000);
		}

	bool bme280p = bmp280.id == BMP280_CHIP_ID;
	size = sprintf((char *)Data, "BMP280: found %s\n", bme280p ? "BME280" : "BMP280");
	//_____________________

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // SENSOR BMP280
	  atualizarDisplayBMP280();
	  HAL_Delay(2000);
	  atualizarDisplayMPU6050();
	  HAL_Delay(2000);
	  capacity();
	  HAL_Delay(2000);



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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 127;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 72;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim14, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HCSR04_Read (void)
{
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	HAL_Delay(10);  // wait for 10 us
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

	__HAL_TIM_ENABLE_IT(&htim14, TIM_IT_CC1);
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
	{
		if (Is_First_Captured==0) // if the first value is not captured
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
			Is_First_Captured = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured==1)   // if the first is already captured
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			if (IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2-IC_Val1;
			}

			else if (IC_Val1 > IC_Val2)
			{
				Difference = (0xffff - IC_Val1) + IC_Val2;
			}

			Distance = Difference * .034/2;
			Is_First_Captured = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
		}
	}
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_1){
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	}

	else if(GPIO_Pin == GPIO_PIN_4){
		moveServo = ~moveServo;
		openDrip();
	}


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
