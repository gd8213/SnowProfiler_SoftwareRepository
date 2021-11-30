///* USER CODE BEGIN Header */
///**
//  ******************************************************************************
//  * @file           : main.c
//  * @brief          : Main program body
//  ******************************************************************************
//  * @attention
//  *
//  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
//  * All rights reserved.</center></h2>
//  *
//  * This software component is licensed by ST under BSD 3-Clause license,
//  * the "License"; You may not use this file except in compliance with the
//  * License. You may obtain a copy of the License at:
//  *                        opensource.org/licenses/BSD-3-Clause
//  *
//  ******************************************************************************
//  */
///* USER CODE END Header */
///* Includes ------------------------------------------------------------------*/
//#include "main.h"
//
///* Private includes ----------------------------------------------------------*/
///* USER CODE BEGIN Includes */
//
///* USER CODE END Includes */
//
///* Private typedef -----------------------------------------------------------*/
///* USER CODE BEGIN PTD */
//
///* USER CODE END PTD */
//
///* Private define ------------------------------------------------------------*/
///* USER CODE BEGIN PD */
///* USER CODE END PD */
//
///* Private macro -------------------------------------------------------------*/
///* USER CODE BEGIN PM */
//
///* USER CODE END PM */
//
///* Private variables ---------------------------------------------------------*/
//SPI_HandleTypeDef hspi2;
//
//UART_HandleTypeDef huart5;
//USART_HandleTypeDef husart2;
//
///* USER CODE BEGIN PV */
//uint32_t TIMEOUT_DURATION=5000;
//SPI_HandleTypeDef hspi2;
//uint8_t flag=0;
//uint16_t accel_data[8000][3]={0};
///* USER CODE END PV */
//
///* Private function prototypes -----------------------------------------------*/
//void SystemClock_Config(void);
//static void MX_GPIO_Init(void);
//static void MX_SPI2_Init(void);
//static void MX_UART5_Init(void);
//static void MX_USART2_Init(void);
///* USER CODE BEGIN PFP */
//void lsm6dsm_init(void);
//float lsm6dsm_from_fs2g_to_mg(int16_t lsb);
//float lsm6dsm_from_fs500dps_to_mdps(int16_t lsb);
//uint16_t lsm6dsm_read_accel(float* accel_x,float* accel_y,float* accel_z);
//static int32_t lsm6ds3_read(void *handle, uint8_t reg, uint8_t *bufp,uint16_t len);
//static int32_t lsm6ds3_write(void *handle, uint8_t reg,uint8_t *bufp,uint16_t len);
///* USER CODE END PFP */
//
///* Private user code ---------------------------------------------------------*/
///* USER CODE BEGIN 0 */
//
///* USER CODE END 0 */
//
///**
//  * @brief  The application entry point.
//  * @retval int
//  */
//int main(void)
//{
//  /* USER CODE BEGIN 1 */
//
//  /* USER CODE END 1 */
//
//  /* MCU Configuration--------------------------------------------------------*/
//
//  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
//  HAL_Init();
//
//  /* USER CODE BEGIN Init */
//
//  /* USER CODE END Init */
//
//  /* Configure the system clock */
//  SystemClock_Config();
//
//  /* USER CODE BEGIN SysInit */
//
//  /* USER CODE END SysInit */
//
//  /* Initialize all configured peripherals */
//  MX_GPIO_Init();
//  MX_SPI2_Init();
//  MX_UART5_Init();
//  MX_USART2_Init();
//  /* USER CODE BEGIN 2 */
//	uint8_t who_am_i;
//	float acc_x;
//	float acc_y;
//	float acc_z;
//	uint8_t CTRL3_C_reg=0x12;
//	uint8_t CTRL3_C_set_sim=0b00001100;
//	lsm6ds3_write(&hspi2, CTRL3_C_reg, &CTRL3_C_set_sim, 1);
//	uint8_t who_am_i_reg=0x0F;
//
//	// lsm6ds3_read(&hspi2, CTRL3_C_reg, &who_am_i, 1);
//
//	lsm6ds3_read(&hspi2, who_am_i_reg, &who_am_i, 1);
//	i=0;
//  /* USER CODE END 2 */
//
//  /* Infinite loop */
//  /* USER CODE BEGIN WHILE */
//  while (1)
//  {
//	  if(1==flag)
//	  {
//		  i++;
//		  lsm6dsm_read_accel(&acc_x,&acc_y,&acc_z);
//		  accel_data
//		  flag=0;
//
//		  accel_data[i][1]=acc_x;
//		  accel_data[i][2]=acc_y;
//		  accel_data[i][3]=acc_z;
//
//	  }
//    /* USER CODE END WHILE */
//
//    /* USER CODE BEGIN 3 */
//  }
//  /* USER CODE END 3 */
//}
//
///**
//  * @brief System Clock Configuration
//  * @retval None
//  */
//void SystemClock_Config(void)
//{
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//  /** Configure the main internal regulator output voltage
//  */
//  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Initializes the RCC Oscillators according to the specified parameters
//  * in the RCC_OscInitTypeDef structure.
//  */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
//  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
//  RCC_OscInitStruct.MSICalibrationValue = 0;
//  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Initializes the CPU, AHB and APB buses clocks
//  */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
//
//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}
//
///**
//  * @brief SPI2 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_SPI2_Init(void)
//{
//
//  /* USER CODE BEGIN SPI2_Init 0 */
//
//  /* USER CODE END SPI2_Init 0 */
//
//  /* USER CODE BEGIN SPI2_Init 1 */
//
//  /* USER CODE END SPI2_Init 1 */
//  /* SPI2 parameter configuration*/
//  hspi2.Instance = SPI2;
//  hspi2.Init.Mode = SPI_MODE_MASTER;
//  hspi2.Init.Direction = SPI_DIRECTION_1LINE;
//  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
//  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
//  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
//  hspi2.Init.NSS = SPI_NSS_SOFT;
//  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
//  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
//  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
//  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
//  hspi2.Init.CRCPolynomial = 7;
//  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
//  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
//  if (HAL_SPI_Init(&hspi2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN SPI2_Init 2 */
//
//  /* USER CODE END SPI2_Init 2 */
//
//}
//
///**
//  * @brief UART5 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_UART5_Init(void)
//{
//
//  /* USER CODE BEGIN UART5_Init 0 */
//
//  /* USER CODE END UART5_Init 0 */
//
//  /* USER CODE BEGIN UART5_Init 1 */
//
//  /* USER CODE END UART5_Init 1 */
//  huart5.Instance = UART5;
//  huart5.Init.BaudRate = 115200;
//  huart5.Init.WordLength = UART_WORDLENGTH_8B;
//  huart5.Init.StopBits = UART_STOPBITS_1;
//  huart5.Init.Parity = UART_PARITY_NONE;
//  huart5.Init.Mode = UART_MODE_TX_RX;
//  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
//  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
//  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
//  if (HAL_UART_Init(&huart5) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN UART5_Init 2 */
//
//  /* USER CODE END UART5_Init 2 */
//
//}
//
///**
//  * @brief USART2 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_USART2_Init(void)
//{
//
//  /* USER CODE BEGIN USART2_Init 0 */
//
//  /* USER CODE END USART2_Init 0 */
//
//  /* USER CODE BEGIN USART2_Init 1 */
//
//  /* USER CODE END USART2_Init 1 */
//  husart2.Instance = USART2;
//  husart2.Init.BaudRate = 115200;
//  husart2.Init.WordLength = USART_WORDLENGTH_8B;
//  husart2.Init.StopBits = USART_STOPBITS_1;
//  husart2.Init.Parity = USART_PARITY_NONE;
//  husart2.Init.Mode = USART_MODE_TX_RX;
//  husart2.Init.CLKPolarity = USART_POLARITY_LOW;
//  husart2.Init.CLKPhase = USART_PHASE_1EDGE;
//  husart2.Init.CLKLastBit = USART_LASTBIT_DISABLE;
//  if (HAL_USART_Init(&husart2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN USART2_Init 2 */
//
//  /* USER CODE END USART2_Init 2 */
//
//}
//
///**
//  * @brief GPIO Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_GPIO_Init(void)
//{
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//  /* GPIO Ports Clock Enable */
//  __HAL_RCC_GPIOD_CLK_ENABLE();
//  __HAL_RCC_GPIOC_CLK_ENABLE();
//  __HAL_RCC_GPIOG_CLK_ENABLE();
//  HAL_PWREx_EnableVddIO2();
//  __HAL_RCC_GPIOB_CLK_ENABLE();
//  __HAL_RCC_GPIOA_CLK_ENABLE();
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_RESET);
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
//
//  /*Configure GPIO pin : PG12 */
//  GPIO_InitStruct.Pin = GPIO_PIN_12;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
//
//  /*Configure GPIO pin : PWMI_RAS_Pin */
//  GPIO_InitStruct.Pin = PWMI_RAS_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(PWMI_RAS_GPIO_Port, &GPIO_InitStruct);
//
//  /*Configure GPIO pin : PC6 */
//  GPIO_InitStruct.Pin = GPIO_PIN_6;
//  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//
//  /*Configure GPIO pin : PB12 */
//  GPIO_InitStruct.Pin = GPIO_PIN_12;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//
//  /* EXTI interrupt init*/
//  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
//
//}
//
///* USER CODE BEGIN 4 */
//
//void lsm6dsm_init(void)
//{
//	uint32_t len =1;
//	uint8_t bufp;
//	// LSM6DS3H_REG_CTRL3_C
//	// set 3-wire SPI mode
//	bufp=0b00001100;
//	uint8_t reg=LSM6DS3H_REG_CTRL3_C;
//	lsm6ds3_write(&hspi2,reg, &bufp, len);
//
//	// LSM6DS3H_REG_CTRL1_XL
//	// Values for acceleration
//	// ODR_XL set to 3.33kHz
//	// FS of accelerometer set to +- 2g
//	// BW0_XL BW set to 1.5kHz
//	bufp=0b10010000;
//	reg=LSM6DS3H_REG_CTRL1_XL;
//	lsm6ds3_write(&hspi2, reg, &bufp, len);
//
//	// LSM6DS3H_REG_CTRL2_G
//	// Values for gyro
//	// ODR_XL set to 3.33kHz
//	// FS of gyro set to 500dps
//	// BW0_XL BW set to 1.5kHz
//	bufp=0b1001010;
//	reg=LSM6DS3H_REG_CTRL2_G;
//	lsm6ds3_write(&hspi2, reg, &bufp, len);
//
//	// LSM6DS3H_REG_CTRL4_C
//	// disable I2C
//	bufp=0b00000000;
//	reg=LSM6DS3H_REG_CTRL4_C;
//	lsm6ds3_write(&hspi2, reg, &bufp, len);
//
//	// LSM6DS3H_REG_CTRL5_C
//	// round values
//	bufp=0b01100000;
//	reg=LSM6DS3H_REG_CTRL5_C;
//	lsm6ds3_write(&hspi2, reg, &bufp, len);
//
//}
//
//float lsm6dsm_from_fs2g_to_mg(int16_t lsb)
//{
//	//
//  return ((float)lsb * 0.061f);
//}
//
//float lsm6dsm_from_fs500dps_to_mdps(int16_t lsb)
//{
//  return ((float)lsb * 17.50f);
//}
//
//
//uint16_t lsm6dsm_read_accel(float* accel_x,float* accel_y,float* accel_z)
//{
//	uint32_t len=1;
//	uint8_t data_H=0x00;
//	uint8_t data_L=0x00;
//	int16_t data=0;
//	float temporary;
//
//	// read x acceleration
//	uint8_t reg=LSM6DS3H_REG_OUTX_H_XL;
//	lsm6ds3_read(&hspi2, reg, &data_H, len);
//	reg=LSM6DS3H_REG_OUTX_L_XL;
//	lsm6ds3_read(&hspi2, reg, &data_L, len);
//	data=(data_H<<8)|data_L;
//	// calc to mg
//	temporary=lsm6dsm_from_fs2g_to_mg(data);
//	// output measurement
//	*accel_x=temporary;
//
//	data_H=0x00;
//	data_L=0x00;
//
//	// read y acceleration
//	reg=LSM6DS3H_REG_OUTY_H_XL;
//	lsm6ds3_read(&hspi2, reg, &data_H, len);
//	reg=LSM6DS3H_REG_OUTY_L_XL;
//	lsm6ds3_read(&hspi2, reg, &data_L, len);
//	data=(data_H<<8)|data_L;
//
//	temporary=lsm6dsm_from_fs2g_to_mg(data);
//	*accel_y=temporary;
//
//	data_H=0x00;
//	data_L=0x00;
//
//	// read z acceleration
//	reg=LSM6DS3H_REG_OUTZ_H_XL;
//	lsm6ds3_read(&hspi2, reg, &data_H, len);
//	reg=LSM6DS3H_REG_OUTZ_L_XL;
//	lsm6ds3_read(&hspi2, reg, &data_L, len);
//	data=(data_H<<8)|data_L;
//
//	temporary=lsm6dsm_from_fs2g_to_mg(data);
//	*accel_z=temporary;
//
//	return 0;
//
//}
//
//uint16_t lsm6dsm_read_gyro(float* gyro_x,float* gyro_y,float* gyro_z)
//{
//	uint32_t len=1;
//	uint8_t data_H;
//	uint8_t data_L;
//	int16_t data=0;
//	float temporary;
//
//	uint8_t reg=LSM6DS3H_REG_OUTX_H_G;
//	lsm6ds3_read(&hspi2, reg, &data_H, len);
//	reg=LSM6DS3H_REG_OUTX_L_G;
//	lsm6ds3_read(&hspi2, reg, &data_L, len);
//	data=(data_H<<8)|data_L;
//
//	temporary=lsm6dsm_from_fs500dps_to_mdps(data);
//
//	*gyro_x=temporary;
//
//	reg=LSM6DS3H_REG_OUTY_H_G;
//	lsm6ds3_read(&hspi2, reg, &data_H, len);
//	reg=LSM6DS3H_REG_OUTY_L_G;
//	lsm6ds3_read(&hspi2, reg, &data_L, len);
//	data=(data_H<<8)|data_L;
//
//	temporary=lsm6dsm_from_fs500dps_to_mdps(data);
//
//	*gyro_y=temporary;
//
//	reg=LSM6DS3H_REG_OUTZ_H_G;
//	lsm6ds3_read(&hspi2, reg, &data_H, len);
//	reg=LSM6DS3H_REG_OUTZ_L_G;
//	lsm6ds3_read(&hspi2, reg, &data_L, len);
//	data=(data_H<<8)|data_L;
//
//	temporary=lsm6dsm_from_fs500dps_to_mdps(data);
//
//	*gyro_z=temporary;
//
//	return 0;
//
//}
//
//static int32_t lsm6ds3_read(void *handle, uint8_t reg, uint8_t *bufp,uint16_t len)
//{
//	reg |= 0x80; // set to one for read operation
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
//	HAL_SPI_Transmit(handle, &reg, 1, TIMEOUT_DURATION);
//	HAL_SPI_Receive(handle, bufp, len, TIMEOUT_DURATION);
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
//
//  return 0;
//}
//
//static int32_t lsm6ds3_write(void *handle, uint8_t reg,uint8_t *bufp,uint16_t len)
//{
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
//	HAL_SPI_Transmit(handle, &reg, 1, TIMEOUT_DURATION);
//	HAL_SPI_Transmit(handle, bufp, len, TIMEOUT_DURATION);
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
//
//  return 0;
//}
//
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//	flag=1;
//}
//
///* USER CODE END 4 */
//
///**
//  * @brief  This function is executed in case of error occurrence.
//  * @retval None
//  */
//void Error_Handler(void)
//{
//  /* USER CODE BEGIN Error_Handler_Debug */
//  /* User can add his own implementation to report the HAL error return state */
//  __disable_irq();
//  while (1)
//  {
//  }
//  /* USER CODE END Error_Handler_Debug */
//}
//
//#ifdef  USE_FULL_ASSERT
///**
//  * @brief  Reports the name of the source file and the source line number
//  *         where the assert_param error has occurred.
//  * @param  file: pointer to the source file name
//  * @param  line: assert_param error line source number
//  * @retval None
//  */
//void assert_failed(uint8_t *file, uint32_t line)
//{
//  /* USER CODE BEGIN 6 */
//  /* User can add his own implementation to report the file name and line number,
//     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//  /* USER CODE END 6 */
//}
//#endif /* USE_FULL_ASSERT */
//
///************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
