//#include "SPI_user.h"
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
