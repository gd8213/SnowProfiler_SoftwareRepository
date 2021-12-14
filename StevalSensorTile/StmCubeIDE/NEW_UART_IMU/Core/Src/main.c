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
#include <stdio.h>
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
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart5;
USART_HandleTypeDef husart2;

/* USER CODE BEGIN PV */

uint32_t Timeout=10000;
SPI_HandleTypeDef hspi2;
HAL_StatusTypeDef uart;
HAL_StatusTypeDef status_spi;
uint32_t TIMEOUT_DURATION=5000;
uint8_t flag=1;
uint8_t rxBuffer[7]={0};
uint8_t aTxBuffer[]="hello\r\n";
uint16_t i=0;
uint8_t SPI_ready=1;
int16_t accel_data_x[4096]={0};
int16_t accel_data_y[4096]={0};
int16_t accel_data_z[4096]={0};
uint32_t time[4096]={0};
static int16_t data_raw_acceleration[3];
float acc_x;
float acc_y;
float acc_z;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_UART5_Init(void);
static void MX_USART2_Init(void);
/* USER CODE BEGIN PFP */
void lsm6dsm_init(void);
float lsm6dsm_from_fs2g_to_mg(int16_t lsb);
float lsm6dsm_from_fs4g_to_mg(int16_t lsb);
float lsm6dsm_from_fs8g_to_mg(int16_t lsb);
float lsm6dsm_from_fs16g_to_mg(int16_t lsb);
float lsm6dsm_from_fs500dps_to_mdps(int16_t lsb);
uint16_t lsm6dsm_read_accel(int16_t* accel_x,int16_t* accel_y,int16_t* accel_z);
int32_t lsm6dsm_acceleration_raw_get(int16_t *val);
static int32_t lsm6ds3_read(void *handle, uint8_t reg, uint8_t *bufp,uint16_t len);
static int32_t lsm6ds3_write(void *handle, uint8_t reg,uint8_t *bufp,uint16_t len);
void whoami(void);
static void Error_Handler_1(void);
uint8_t isKthBitSet(int n, int k);
uint8_t sftwRESET(void);
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
  MX_SPI2_Init();
  MX_UART5_Init();
  MX_USART2_Init();
  /* USER CODE BEGIN 2 */
	Error_Handler_1();

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

	lsm6dsm_init();
	HAL_Delay(200);

	whoami(); // check if device can be found

	// set UART5 interrupt
	HAL_UART_Receive_IT(&huart5, rxBuffer, 7);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_SET); // Switch on LED
	i=0;
	uint8_t result=0;
  	uint8_t bufp=0x00;
  	uint8_t reg=0x1e; // register of IMU where DRDY signal can be found

  while (1)
  {
	  // polling data
	  if(1) // flag
	  {

		  // check if new data is available
		  	lsm6ds3_read(&hspi2, reg, &bufp, 1);
		  	result=isKthBitSet(bufp, 1);

		  	if(result)
		  	{
		  		// memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
		  		// interrupt if PWM occurs
		  		if(i<4096)
		  		{

		  			lsm6dsm_acceleration_raw_get(data_raw_acceleration);
		  			acc_x=lsm6dsm_from_fs4g_to_mg(data_raw_acceleration[0]);
		  			acc_y=lsm6dsm_from_fs4g_to_mg(data_raw_acceleration[1]);
		  			acc_z=lsm6dsm_from_fs4g_to_mg(data_raw_acceleration[2]);
		  			accel_data_x[i]=(int16_t)acc_x;
		  			accel_data_y[i]=(int16_t)acc_y;
		  			accel_data_z[i]=(int16_t)acc_z;
		  			// time[i]=HAL_GetTick();
		  			i++;
		  			flag=0;

		  		}
		  		else{
		  			i=0;
		  		}

		  	}
	  }

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
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PG12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PWMI_RAS_Pin */
  GPIO_InitStruct.Pin = PWMI_RAS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PWMI_RAS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_IMU_Pin */
  GPIO_InitStruct.Pin = CS_IMU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_IMU_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
uint8_t sftwRESET(void)
{
	uint32_t len =1;
	uint8_t bufp;
	uint8_t read_reg;
	// LSM6DS3H_REG_CTRL3_C
	// set 3-wire SPI mode
	// set block data update
	bufp=0b11111110;
	uint8_t reg=LSM6DS3H_REG_CTRL3_C;
	lsm6ds3_read(&hspi2, reg, &read_reg, len);
	bufp&=read_reg;
	lsm6ds3_write(&hspi2,reg, &bufp, len);
}


void whoami(void)
{
	// adr_WHO_AM_I has to be 0x6a
	uint8_t who_am_i=0x00;
	uint8_t who_am_i_reg=0x0f;

	lsm6ds3_read(&hspi2, who_am_i_reg, &who_am_i, 1);

	if(who_am_i!=0x6a)
	{
		// error when jumping in here
		// device not found!!!
		// possible problems can be wrong DEBUG port is used
		while(1)
		{
			lsm6ds3_read(&hspi2, who_am_i_reg, &who_am_i, 1);
			if(who_am_i==0x6a){break;}
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// __NOP(); // used to debug the Callback
	int size;
	char data_s[256];
	uint16_t length_data_arry=4096;

	// for(int j=0; j<length_data_arry;j++)
	for(int j=0; j<i;j++)
	{
		size=sprintf(data_s, "%05d\n",accel_data_z[j]);
		HAL_UART_Transmit(&huart5,(uint8_t *)data_s, size, Timeout);
	//		size = sprintf(data_s, "X: %d,Y: %d,Z :%d\r\n",accel_data_x[j],accel_data_y[j],accel_data_z[j]);
	//		HAL_UART_Transmit(&huart5,(uint8_t *)data_s, size, Timeout);
	}
	i=0;
	HAL_UART_Receive_IT(&huart5, rxBuffer, 7);
}

static void Error_Handler_1(void)
{
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_Delay(400);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_Delay(400);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_Delay(400);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_Delay(400);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_Delay(400);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_RESET);
}

void lsm6dsm_init(void)
{
	uint32_t len =1;
	uint8_t bufp;
	uint8_t read_reg;
	// LSM6DS3H_REG_CTRL3_C
	// set 3-wire SPI mode
	// set block data update
	bufp=BDU|SIM|IF_INC; //0b01001100;
	uint8_t reg=LSM6DS3H_REG_CTRL3_C;
	lsm6ds3_write(&hspi2,reg, &bufp, len);

	// LSM6DS3H_REG_CTRL1_XL
	// Values for acceleration
	// ODR_XL set to 6.66kHz
	// FS of accelerometer set to +- 4g
	// BW0_XL BW set to 400Hz
	bufp=ODR_6660Hz|FS_4g; // 0b10101000;
	reg=LSM6DS3H_REG_CTRL1_XL;
	lsm6ds3_write(&hspi2, reg, &bufp, len);

	// disable latched mode
	bufp=DRDY_LATCHED;
	reg=LSM6DS3H_REG_DRDY_PULSE_CFG;
	lsm6ds3_write(&hspi2, reg, &bufp, len);

}

float lsm6dsm_from_fs2g_to_mg(int16_t lsb)
{
  return ((float)lsb * 0.061f);
}

float lsm6dsm_from_fs4g_to_mg(int16_t lsb)
{
  return ((float)lsb * 0.122f);
}

float lsm6dsm_from_fs8g_to_mg(int16_t lsb)
{
  return ((float)lsb * 0.244f);
}

float lsm6dsm_from_fs16g_to_mg(int16_t lsb)
{
  return ((float)lsb * 0.488f);
}

float lsm6dsm_from_fs500dps_to_mdps(int16_t lsb)
{
  return ((float)lsb * 17.50f);
}

uint8_t isKthBitSet(int n, int k)
{
    if (n & (1 << (k - 1)))
        {return 1;}
    else
        {return 0;}
}


uint16_t lsm6dsm_read_accel(int16_t* accel_x,int16_t* accel_y,int16_t* accel_z)
{
	// STM solution begin ##################################################
//	int16_t val[3]={0};
//	uint8_t buff[6];
//	  int32_t ret;
//	  float temporary;
//	  uint8_t reg=LSM6DS3H_REG_OUTX_L_XL;
//	  lsm6ds3_read(&hspi2, reg, &buff[0], 6);
//	  val[0] = (int16_t)buff[1];
//	  val[0] = (val[0] * 256) + (int16_t)buff[0];
//	  val[1] = (int16_t)buff[3];
//	  val[1] = (val[1] * 256) + (int16_t)buff[2];
//	  val[2] = (int16_t)buff[5];
//	  val[2] = (val[2] * 256) + (int16_t)buff[4];
//	  temporary=lsm6dsm_from_fs2g_to_mg(val[0]);
//	  *accel_x=(int16_t)temporary;
//	  temporary=lsm6dsm_from_fs2g_to_mg(val[1]);
//	  *accel_y=(int16_t)temporary;
//	  temporary=lsm6dsm_from_fs2g_to_mg(val[2]);
//	  *accel_z=(int16_t)temporary;
	// STM solution ends  ##################################################
	SPI_ready=0;
	uint32_t len=1;
	uint8_t data_H=0x00;
	uint8_t data_L=0x00;
	int16_t data=0;
	float temporary;

	// check if new data is available
	uint8_t bufp=0x00;
	uint8_t reg=0x1e;
	lsm6ds3_read(&hspi2, reg, &bufp, len);

	if(isKthBitSet(bufp,1))
	{

		data_H=0x00;
		data_L=0x00;
		// read z acceleration
		reg=LSM6DS3H_REG_OUTZ_H_XL;
		lsm6ds3_read(&hspi2, reg, &data_H, len);
		reg=LSM6DS3H_REG_OUTZ_L_XL;
		lsm6ds3_read(&hspi2, reg, &data_L, len);
		data=(data_H<<8)|data_L;

		temporary=lsm6dsm_from_fs2g_to_mg(data);
		*accel_z=temporary;
		if(i==100)
		{
			// only for degugging
		}
		// read x acceleration
		reg=LSM6DS3H_REG_OUTX_H_XL;
		lsm6ds3_read(&hspi2, reg, &data_H, len);
		reg=LSM6DS3H_REG_OUTX_L_XL;
		lsm6ds3_read(&hspi2, reg, &data_L, len);
		data=(data_H<<8)|data_L;
		// calc to mg
		temporary=lsm6dsm_from_fs2g_to_mg(data);
		// output measurement
		*accel_x=temporary;

		data_H=0x00;
		data_L=0x00;

		// read y acceleration
		reg=LSM6DS3H_REG_OUTY_H_XL;
		lsm6ds3_read(&hspi2, reg, &data_H, len);
		reg=LSM6DS3H_REG_OUTY_L_XL;
		lsm6ds3_read(&hspi2, reg, &data_L, len);
		data=(data_H<<8)|data_L;

		temporary=lsm6dsm_from_fs2g_to_mg(data);
		*accel_y=temporary;

	}
	else
	{
		*accel_x=9999;
		*accel_y=9999;
		*accel_z=9999;
	}
	SPI_ready=1;
	return 0;

}

int32_t lsm6dsm_acceleration_raw_get(int16_t *val)
{
  uint8_t buff[6];
  int32_t ret;

  lsm6ds3_read(&hspi2, LSM6DS3H_REG_OUTX_L_XL, &buff, 6);
  val[0] = (int16_t)buff[1];
  val[0] = (val[0] * 256) + (int16_t)buff[0];
  val[1] = (int16_t)buff[3];
  val[1] = (val[1] * 256) + (int16_t)buff[2];
  val[2] = (int16_t)buff[5];
  val[2] = (val[2] * 256) + (int16_t)buff[4];

  return ret;
}

uint16_t lsm6dsm_read_gyro(float* gyro_x,float* gyro_y,float* gyro_z)
{
	uint32_t len=1;
	uint8_t data_H;
	uint8_t data_L;
	int16_t data=0;
	float temporary;

	uint8_t reg=LSM6DS3H_REG_OUTX_H_G;
	lsm6ds3_read(&hspi2, reg, &data_H, len);
	reg=LSM6DS3H_REG_OUTX_L_G;
	lsm6ds3_read(&hspi2, reg, &data_L, len);
	data=(data_H<<8)|data_L;

	temporary=lsm6dsm_from_fs500dps_to_mdps(data);

	*gyro_x=temporary;

	reg=LSM6DS3H_REG_OUTY_H_G;
	lsm6ds3_read(&hspi2, reg, &data_H, len);
	reg=LSM6DS3H_REG_OUTY_L_G;
	lsm6ds3_read(&hspi2, reg, &data_L, len);
	data=(data_H<<8)|data_L;

	temporary=lsm6dsm_from_fs500dps_to_mdps(data);

	*gyro_y=temporary;

	reg=LSM6DS3H_REG_OUTZ_H_G;
	lsm6ds3_read(&hspi2, reg, &data_H, len);
	reg=LSM6DS3H_REG_OUTZ_L_G;
	lsm6ds3_read(&hspi2, reg, &data_L, len);
	data=(data_H<<8)|data_L;

	temporary=lsm6dsm_from_fs500dps_to_mdps(data);

	*gyro_z=temporary;

	return 0;

}

static int32_t lsm6ds3_read(void *handle, uint8_t reg, uint8_t *bufp,uint16_t len)
{
	reg |= 0x80; // set MSB to one for read operation
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	status_spi=HAL_SPI_Transmit(handle, &reg, 1, TIMEOUT_DURATION);
	status_spi=HAL_SPI_Receive(handle, bufp, len, TIMEOUT_DURATION);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	while(status_spi!=HAL_OK)
	{
		// spi communication isnt finished or something failed
		return -1;
	}
  return 0;
}

static int32_t lsm6ds3_write(void *handle, uint8_t reg,uint8_t *bufp,uint16_t len)
{
	reg |= 0x00;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	status_spi=HAL_SPI_Transmit(handle, &reg, 1, TIMEOUT_DURATION);
	status_spi=HAL_SPI_Transmit(handle, bufp, len, TIMEOUT_DURATION);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

  return 0;
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	flag=1;
//	memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
//	// interrupt if PWM occurs
//	if(i<4096)
//	{
//		lsm6dsm_acceleration_raw_get(data_raw_acceleration);
//		// time[i]=HAL_GetTick(); // check PWM frequency
//		// lsm6dsm_read_accel(&acc_x,&acc_y,&acc_z);
////		accel_data_x[i]=(int16_t)acc_x;
////		accel_data_y[i]=(int16_t)acc_y;
////		accel_data_z[i]=(int16_t)acc_z;
//		acc_x=lsm6dsm_from_fs4g_to_mg(data_raw_acceleration[0]);
//		acc_y=lsm6dsm_from_fs4g_to_mg(data_raw_acceleration[1]);
//		acc_z=lsm6dsm_from_fs4g_to_mg(data_raw_acceleration[2]);
//		accel_data_x[i]=(int16_t)acc_x;
//		accel_data_y[i]=(int16_t)acc_y;
//		accel_data_z[i]=(int16_t)acc_z;
//		i++;
//	}
//	else{i=0;}
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
