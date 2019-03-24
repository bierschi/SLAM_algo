/*
 * PeripheralHandles.h
 *
 *  Created on: 04.06.2016
 *      Author: stefan
 */

#ifndef INC_PERIPHERALHANDLES_H_
#define INC_PERIPHERALHANDLES_H_

#include "stm32f3xx_hal.h"
#include "PIDControl.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern PIDC_ControllerHandleType MainEngine;
extern UART_HandleTypeDef huart3;
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;

#endif /* INC_PERIPHERALHANDLES_H_ */
