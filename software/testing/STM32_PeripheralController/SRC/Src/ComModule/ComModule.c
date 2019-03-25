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

ComStructureType COM_StructRX;
ComStructureType COM_StructTX;


void COM_Init(void)
{
	ComStructureType zero = {0};

	// just receive for the moment
	HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *) &COM_StructTX, (uint8_t *) &COM_StructRX, sizeof(ComStructureType));

	COM_StructRX = zero;
	COM_StructTX = zero;

	COM_StructRX.CurrentSteeringMode = COM_STEERING_MODE_MANUAL;
	COM_StructRX.CurrentSteeringAngle = COM_STEERING_ANGLE_ZERO;
	COM_StructRX.CurrentSteeringDirection = COM_STEERING_DIRECTON_ZERO;
	COM_StructRX.CurrentSteeringSpeed = COM_STEERING_SPEED_ZERO;

	COM_StructRX.Target_X = 50.0f;
	COM_StructRX.Target_Y = 0.0f;

}

void COM_PrintToUART(uint8_t *data, uint16_t size)
{
    HAL_UART_Transmit(&huart3, data, size, 9u);
}
