/*
 * ComModule.c
 *
 *  Created on: 12.06.2016
 *      Author: stefan
 */

#include "ComModule.h"
#include "ComStructure.h"
#include "PeripheralHandles.h"
#include "PlannerModule.h"

ComStructureType COM_Struct;


void COM_Init(void)
{
	// just receive for the moment
    HAL_SPI_Receive_DMA(&hspi1, (uint8_t *) &COM_Struct, sizeof(ComStructureType));

    COM_Struct.CurrentSteeringMode = 0u;
    COM_Struct.CurrentSteeringAngle = COM_STEERING_ANGLE_ZERO;
    COM_Struct.CurrentSteeringDirection = COM_STEERING_DIRECTON_ZERO;
    COM_Struct.CurrentSteeringSpeed = COM_STEERING_SPEED_ZERO;

    COM_Struct.Target_X = 0.0f;
    COM_Struct.Target_Y = 0.0f;
}

void COM_PrintToUART(uint8_t *data, uint16_t size)
{
    HAL_UART_Transmit(&huart3, data, size, 9u);
}
