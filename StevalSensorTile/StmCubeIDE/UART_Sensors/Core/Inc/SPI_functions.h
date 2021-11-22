/*
 * SPI_functions.h
 *
 *  Created on: 17.11.2021
 *      Author: Felbermayr Simon
 */

#ifndef INC_SPI_FUNCTIONS_H_
#define INC_SPI_FUNCTIONS_H_

#define TIMEOUT_DURATION 1000

#include "SensorTile.h"
/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

uint8_t readREG(uint8_t adr,uint8_t *data);
void writeREG(uint8_t adr,uint8_t *data);


#endif /* INC_SPI_FUNCTIONS_H_ */
