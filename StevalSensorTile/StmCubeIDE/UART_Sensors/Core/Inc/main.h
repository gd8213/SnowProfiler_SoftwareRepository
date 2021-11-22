/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#define SAMPLING_100Hz
//#define SAMPLING_100Hz

#if defined( SAMPLING_50Hz)
  #define DEFAULT_uhCCR1_Val 190
  #define ACCELERO_ODR 416.0f     /* after filtering will be 46.2 Hz */
  #define ACCELERO_DIV LSM6DSM_ACC_GYRO_HPCF_XL_DIV9
  #define GYRO_ODR 52.0f
  #define MAGNETO_ODR 50.0f
  #define PRESSURE_ODR 50.0f
  #define DATA_PERIOD_MS     (20)

#elif defined( SAMPLING_100Hz)
  #define DEFAULT_uhCCR1_Val 100
  #define ACCELERO_ODR  833.0f    /* after filtering will be 93 Hz */
  #define ACCELERO_DIV LSM6DSM_ACC_GYRO_HPCF_XL_DIV9
  #define GYRO_ODR 104.0f
  #define MAGNETO_ODR 100.0f
  #define PRESSURE_ODR 50.0f
  #define DATA_PERIOD_MS     (10)
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "SensorTile_motion_sensors.h"
#include "lsm6dsm_settings.h"

typedef struct
{
  uint32_t ms_counter;
  float pressure;
  float humidity;
  float temperature;
  BSP_MOTION_SENSOR_Axes_t acc;
  BSP_MOTION_SENSOR_Axes_t gyro;
  BSP_MOTION_SENSOR_Axes_t mag;
} T_SensorsData;

#define TXBUFFERSIZE                      (COUNTOF(aTxBuffer) - 1)
/* Size of Reception buffer */
#define RXBUFFERSIZE                      10

/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* Exported functions ------------------------------------------------------- */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);



#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
