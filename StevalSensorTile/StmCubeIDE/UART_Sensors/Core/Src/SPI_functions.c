

#include "SPI_functions.h"

uint8_t readREG(uint8_t adress,uint8_t *data)
{
	uint8_t read=1;
	uint8_t adr=(read<<7)|adress;
	uint8_t *p=&adr;
	uint8_t *rec;

	HAL_GPIO_WritePin(BSP_LSM6DSM_CS_PORT, BSP_LSM6DSM_CS_PIN, GPIO_PIN_RESET);

	HAL_SPI_Transmit(&hspi2,p, sizeof(adr), TIMEOUT_DURATION);
	if(HAL_SPI_Receive(&hspi2,rec, sizeof(adr), TIMEOUT_DURATION)== HAL_OK)
		{
		return &rec;
		}
	else{
		return 2;
	}

	HAL_GPIO_WritePin(BSP_LSM6DSM_CS_PORT, BSP_LSM6DSM_CS_PIN, GPIO_PIN_SET);
}

void writeREG(uint8_t adress,uint8_t *data)
{
	uint8_t write=0;
	uint8_t* adr=(write<<8)|adress;

	HAL_GPIO_WritePin(BSP_LSM6DSM_CS_PORT, BSP_LSM6DSM_CS_PIN, GPIO_PIN_RESET);

	HAL_SPI_Transmit(&hspi2, &adr, 7, TIMEOUT_DURATION);
	HAL_SPI_Transmit(&hspi2, data, 7, TIMEOUT_DURATION);

	HAL_GPIO_WritePin(BSP_LSM6DSM_CS_PORT, BSP_LSM6DSM_CS_PIN, GPIO_PIN_SET);
}

