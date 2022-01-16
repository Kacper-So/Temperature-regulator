/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BMPXX80.h"
#include "LCD.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef float float32_t;
typedef struct{
	float32_t Kp;
	float32_t Ki;
	float32_t Kd;
	float32_t dt;
	float32_t previous_error, previous_integral;
}pidRegulator; // PID regulator parameters and variables needed to implement discrete PID
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define maxMessageSize 30
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t singleMessageRecived[maxMessageSize]; //message buf for uart recive
uint8_t singleMessageTransmit[maxMessageSize]; //message buf for uart transmit
uint16_t messageSize = 0; //message size for uart
char stringForLCD[maxMessageSize]; //string used to display data on LCD
float temperature;
int setPoint = 0;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

// PID regulator with parameters taken from matlab simulation of regulated process
pidRegulator pidR = { .Kp=0.09486671759812270, .Ki=0.000482928713856668, .Kd=1.50608802557373, .dt=1.0, .previous_error=0, .previous_integral=0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
float32_t calculate_discrete_pid(pid_t* pid, int setPoint, float32_t temperature){
	float32_t u=0, P, I, D, error, integral, derivative;

	error = setPoint-temperature;

	//proportional part
	P = pidR.Kp * error;

	//integral part
	integral = pidR.previous_integral + (error+pidR.previous_error) ;
	pidR.previous_integral = integral;
	I = pidR.Ki*integral*(pidR.dt/2.0);

	//derivative part
	derivative = (error - pidR.previous_error)/pidR.dt;
	pidR.previous_error = error;
	D = pidR.Kd*derivative;

	u = P  + I + D;

	return u;
}

//Uart handling
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart->Instance == USART2){

		char *ptr = singleMessageRecived; //ptr to first char in message

		//if message == ?? then return current setPoint
		if(*ptr == '?'){
			messageSize = sprintf(singleMessageTransmit, "%d", setPoint);
			HAL_UART_Transmit_IT(&huart2, singleMessageTransmit, messageSize);
		} else { //else set new setPoint
			setPoint = atoi(ptr);
		}

		HAL_UART_Receive_IT(&huart2, singleMessageRecived, 2);
	}
}

/*void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	if(huart->Instance == USART2){
		char *ptr = singleMessageRecived;
		if(*ptr == '?'){
			messageSize = sprintf(singleMessageTransmit, "%d", setPoint);
			HAL_UART_Transmit_IT(&huart2, singleMessageTransmit, messageSize);
		} else {
			setPoint = atoi(ptr);
			messageSize = sprintf(singleMessageTransmit, "%d", setPoint);
			HAL_UART_Transmit_IT(&huart2, singleMessageTransmit, messageSize);
		}
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, singleMessageRecived, maxMessageSize);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
	}
}*/

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM3){
		temperature = BMP280_ReadTemperature();                                       //reading temp from sensor

		sprintf(singleMessageTransmit, "%f", temperature);                            //temp to list of uint8_t conversion
		sprintf(stringForLCD, "%s\0", singleMessageTransmit);                         //list of uint8_t to string conversion
		LCD_goto_xy(0,11);                                                            //display curr temp on LCD
		LCD_write_text(stringForLCD);

		sprintf(singleMessageTransmit, "%d", setPoint);                               //setPoint to list of uint8_t conversion
		sprintf(stringForLCD, "%s\0", singleMessageTransmit);                         //list of uint8_t to string conversion
		LCD_goto_xy(1,11);                                                            //display setPoint on LCD
		LCD_write_text(stringForLCD);

		//calculating duty for PWM signal with PID regulator
		float dutyF = (999.0 * calculate_discrete_pid(&pidR, setPoint, temperature));
		uint16_t duty = 0;
		//saturation implementation
		if(dutyF < 0 ) duty = 0; else
		if(dutyF > 999.0) duty = 999; else
			duty = (uint16_t)dutyF;
		//set curr duty
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,duty);
	}
}
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
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  //HAL_UARTEx_ReceiveToIdle_DMA(&huart2, singleMessageRecived, maxMessageSize);
  //__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
  HAL_UART_Receive_IT(&huart2, singleMessageRecived, 2); //uart interrupt init
  HAL_TIM_Base_Start_IT(&htim3);                         //dt tim interrupt init
  LCD_init();                                            //LCD init + setting initial layout
  LCD_goto_xy(0,0);
  LCD_write_text("CurTemp : ");
  LCD_goto_xy(1,0);
  LCD_write_text("RefTemp : ");
  BMP280_Init(&hi2c1, BMP280_TEMPERATURE_16BIT, BMP280_STANDARD, BMP280_FORCEDMODE); //temp sensor init
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);                                          //PWM init
  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);                                     //init duty = 0
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

/* USER CODE BEGIN 4 */
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
