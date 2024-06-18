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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
const char apn[]  = "web.vodafone.de";
const char host[] = "tcp://test.mosquitto.org";
const int  port = 1883;
const char username[] = "";
const char password[] = "";
const char topic[] = "dh11/temp/1";
const uint32_t timeOut =10000;
char reply[200] = {0};
uint8_t ATisOK = 0;
uint8_t CGREGisOK = 0;
uint8_t MQTTisStart = 0;
uint8_t MQTTisConnect = 0;
uint32_t startTime;
//uint16_t readValue;
char ATC[60];
char TempReading[20];
const char readValue[] = "HELLO";
static const uint8_t TMP_ADDR = 0x67 << 1;
static const uint8_t TMP_REG = 0x00;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void powerOn(void);
void SIMTransmit(char *cmd);
void startMQTT(void);
void transmitMQTT(void);
void endMQTT(void);
void mainMQTT(void);
void readTemperature(void);
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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  powerOn();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  readTemperature();
	  mainMQTT();
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
  hi2c1.Init.ClockSpeed = 100000;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief SIM7600 power on
  * @retval None
  */
void powerOn() {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable GPIO Clock */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

    /* Configure GPIO pin : PA10 */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Set the pin to HIGH */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_Delay(500);

    /* Set the pin to LOW */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
}

/**
  * @brief AT command transmit
  * @retval None
  */
void SIMTransmit(char *cmd){
  memset(reply,0,sizeof(reply));
  HAL_UART_Transmit(&huart1,(uint8_t *)cmd, strlen(cmd), 500);
  HAL_UART_Receive (&huart1, (uint8_t*)reply, sizeof(reply), 500);
  HAL_Delay(500);
  HAL_UART_Transmit(&huart2, (uint8_t*)reply, strlen(reply), 500);
}

/**
  * @brief MQTT start and initializing
  * @retval None
  */
void startMQTT(void){
  ATisOK = 0;
  CGREGisOK = 0;
  MQTTisStart = 0;
  MQTTisConnect = 0;

  startTime = HAL_GetTick();
  while(!ATisOK && startTime + timeOut > HAL_GetTick()){
	  SIMTransmit("AT\r\n");
	  HAL_Delay(100);
	  if(strstr((char *)reply, "OK")){
		  ATisOK = 1;
	  }
  }
  if(ATisOK){
	  startTime = HAL_GetTick();
	  while(!CGREGisOK && startTime + timeOut > HAL_GetTick()){
	 	  SIMTransmit("AT+CGREG?\r\n");
	 	  HAL_Delay(100);
	 	  if(strstr((char *)reply, "+CGREG: 0,1")){
	 		 CGREGisOK = 1;
	 	  }
	   }
  }
  if(CGREGisOK ){
	  SIMTransmit("AT+CMQTTSTART\r\n");
	  if (strstr((char *)reply, "ERROR")){
		  SIMTransmit("AT+CMQTTDISC=0,120\r\n");
		  SIMTransmit("AT+CMQTTREL=0\r\n");
		  SIMTransmit("AT+CMQTTSTOP\r\n");
		  }
	  else {
		  MQTTisStart = 1;
		  SIMTransmit("AT+CMQTTACCQ=0,\"client01\"\r\n");
		  sprintf(ATC,"AT+CMQTTCONNECT=0,\"%s:%d\",60,1\r\n",host,port);
		  SIMTransmit(ATC);
		  if(strstr((char *)reply, "OK")){
			 MQTTisConnect = 1;
		  }
	  }
   }
}


/**
  * @brief MQTT data publish
  * @retval None
  */
void transmitMQTT(void){
	sprintf(ATC, "AT+CMQTTTOPIC=0,%d\r\n", strlen(topic));
	SIMTransmit(ATC);
	SIMTransmit("dh11/temp/1\r\n");
	sprintf(ATC, "AT+CMQTTPAYLOAD=0,%d\r\n", strlen(TempReading));
	SIMTransmit(ATC);
	SIMTransmit(TempReading);
	SIMTransmit("AT+CMQTTPUB=0,1,60\r\n");
}

/**
  * @brief MQTT terminate
  * @retval None
  */
void endMQTT(void) {
	if (MQTTisConnect){
		SIMTransmit("AT+CMQTTDISC=0,120\r\n");
		if(!strstr((char *)reply, "OK")){
			return;
		}

		SIMTransmit("AT+CMQTTREL=0\r\n");
		if(!strstr((char *)reply, "OK")){
			return;
		}
	}

	if (MQTTisStart) {
		SIMTransmit("AT+CMQTTSTOP\r\n");
		if(!strstr((char *)reply, "OK")){
			return;
		}
	}

	MQTTisStart = 0;
	MQTTisConnect = 0;
}

/**
  * @brief main MQTT loop
  * @retval None
  */
void mainMQTT(void){
	if (!MQTTisStart || !MQTTisConnect) {
	        startMQTT();
	    }

	if (MQTTisStart && MQTTisConnect) {
	        transmitMQTT();
	    }

}

/**
  * @brief Temperature read
  * @retval None
  */
void readTemperature(void) {
	HAL_StatusTypeDef ret;
	uint8_t buf[30];
	int16_t val;
	float tempt;
	buf[0] = TMP_REG;
	ret = HAL_I2C_Master_Transmit(&hi2c1, TMP_ADDR, buf, 1, HAL_MAX_DELAY);
	if (ret != HAL_OK) {
		strcpy((char*)buf, "Error Communicating Tx\r\n");
		}
	else {
		ret = HAL_I2C_Master_Receive(&hi2c1, TMP_ADDR, buf, 2, HAL_MAX_DELAY);
		if (ret != HAL_OK) {
			strcpy((char*)buf, "Error Communicating Rx\r\n");
		} else {
			val = ((int16_t)buf[0] << 8) | (buf[1]);
			if ( val > 0x7FF) {
				val |= 0xF000;
			}
			tempt = val * 0.0625;
			tempt *= 100;
			sprintf(TempReading, "%u.%u C\r\n", ((unsigned int)tempt / 100), ((unsigned int)tempt % 100));
			}
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
