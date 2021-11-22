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
#include "SensorTile.h"
#include "SensorTile.c"
#include "SPI_functions.h"

UART_HandleTypeDef huart5;
USART_HandleTypeDef husart2;

LSM6DSM_Object_t lsm6dsm;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_UART5_Init(void);
static void MX_USART2_Init(void);
static void Error_Handler_1(void);
int32_t getSensorsData( T_SensorsData *mptr);



uint8_t aTxBuffer[] = "Hello World this message will be send over and over\r\n";
/* Buffer used for reception */
uint8_t aRxBuffer[RXBUFFERSIZE];
T_SensorsData *mptr;

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();


  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_UART5_Init();
  MX_USART2_Init();
  Sensor_IO_SPI_CS_Init_All();
//   MX_X_CUBE_MEMS1_Init();

	BSP_LED_Init(LED1);
	BSP_LED_On(LED1);

	uint8_t *error;
	uint8_t who_am_i;

	uint8_t CTRL3_C_reg=0x12;
	uint8_t CTRL3_C=0b00001100;
	uint16_t transmit=(CTRL3_C_reg<<8)|CTRL3_C;
	uint16_t *CTRL3_C_pointer=&transmit;

	uint8_t reg=0b10001111;
	uint8_t *pointer=&reg;
	uint8_t who_am_i_reg=0x0F;
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//	HAL_GPIO_WritePin(BSP_LSM6DSM_CS_PORT, BSP_LSM6DSM_CS_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(BSP_LSM6DSM_CS_PORT, BSP_LSM6DSM_CS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2,CTRL3_C_pointer, 2, TIMEOUT_DURATION);
	HAL_GPIO_WritePin(BSP_LSM6DSM_CS_PORT, BSP_LSM6DSM_CS_PIN, GPIO_PIN_SET);

  while (1)
  {
	// HAL_SPI_TransmitReceive(&hspi2,(uint8_t *) reg, who_am_i, 2, TIMEOUT_DURATION);
	// HAL_SPI_TransmitReceive(&hspi2, &reg, &who_am_i_reg, 2, TIMEOUT_DURATION);
	HAL_GPIO_WritePin(BSP_LSM6DSM_CS_PORT, BSP_LSM6DSM_CS_PIN, GPIO_PIN_RESET);
//	HAL_SPI_Transmit(&hspi2,&reg, 1, TIMEOUT_DURATION);
//	HAL_SPI_Receive(&hspi2,&who_am_i, 1, TIMEOUT_DURATION);

	HAL_SPI_TransmitReceive(&hspi2, &reg, who_am_i, 2, TIMEOUT_DURATION);
//	BSP_SPI2_SendRecv(&reg, &who_am_i, TIMEOUT_DURATION);
	HAL_GPIO_WritePin(BSP_LSM6DSM_CS_PORT, BSP_LSM6DSM_CS_PIN, GPIO_PIN_SET);

	// error=readREG(who_am_i_reg,pointer);

	HAL_UART_Transmit(&huart5,(uint8_t *)aTxBuffer, TXBUFFERSIZE,5);
	if (HAL_UART_Transmit(&huart5, (uint8_t *)aTxBuffer, TXBUFFERSIZE,5) != HAL_OK)
	  {
			/* Transfer error in transmission process */
			Error_Handler_1();
	  }
	  HAL_Delay(1000);
  }
  /* USER CODE END 3 */
}

static void Error_Handler_1(void)
{
	BSP_LED_Off(LED1);
	HAL_Delay(400);
	BSP_LED_On(LED1);
	HAL_Delay(400);
	BSP_LED_Off(LED1);
	HAL_Delay(400);
	BSP_LED_On(LED1);
	HAL_Delay(400);
	BSP_LED_Off(LED1);
	HAL_Delay(400);
	BSP_LED_On(LED1);
	HAL_Delay(400);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_1LINE;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH; 				// SPI_POLARITY_LOW
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE; 					// SPI_PHASE_1EDGE
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;	// SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE; 				// SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
//  /* USER CODE BEGIN SPI2_Init 2 */
//  HAL_Delay(5);
//  SPI_1LINE_TX(hspi2);
//  HAL_Delay(5);
//  __HAL_SPI_ENABLE(hspi2);
//  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  husart2.Instance = USART2;
  husart2.Init.BaudRate = 115200;
  husart2.Init.WordLength = USART_WORDLENGTH_8B;
  husart2.Init.StopBits = USART_STOPBITS_1;
  husart2.Init.Parity = USART_PARITY_NONE;
  husart2.Init.Mode = USART_MODE_TX_RX;
  husart2.Init.CLKPolarity = USART_POLARITY_LOW;
  husart2.Init.CLKPhase = USART_PHASE_1EDGE;
  husart2.Init.CLKLastBit = USART_LASTBIT_DISABLE;
  if (HAL_USART_Init(&husart2) != HAL_OK)
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

int32_t getSensorsData( T_SensorsData *mptr)
{
  int32_t ret = BSP_ERROR_NONE;
  mptr->ms_counter = HAL_GetTick();

  /* Get Data from Sensors */
  if ( BSP_MOTION_SENSOR_GetAxes(LSM6DSM_0, MOTION_ACCELERO, &mptr->acc ) == BSP_ERROR_COMPONENT_FAILURE )
  {
    mptr->acc.x = 0;
    mptr->acc.y = 0;
    mptr->acc.z = 0;
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }

  if ( BSP_MOTION_SENSOR_GetAxes(LSM6DSM_0, MOTION_GYRO, &mptr->gyro ) == BSP_ERROR_COMPONENT_FAILURE )
  {
    mptr->gyro.x = 0;
    mptr->gyro.y = 0;
    mptr->gyro.z = 0;
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  return ret;
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
