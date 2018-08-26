/*
 * ComModule.c
 *
 *  Created on: 12.06.2016
 *      Author: stefan
 */

#include "ComModule.h"
#include "PeripheralHandles.h"
#include "PlannerModule.h"

uint8_t COM_RxBuffer[COM_RXTX_BUFFER_SIZE];
uint8_t COM_TxBuffer[COM_RXTX_BUFFER_SIZE];
uint8_t COM_CurrentRawRXCommandSet[COM_RXTX_BUFFER_SIZE];
uint8_t COM_rxCommandReady;
uint8_t COM_txCommandReady;

void COM_Init(void)
{
    HAL_UART_Receive_IT(&huart3, COM_RxBuffer, COM_RXTX_BUFFER_SIZE);
    COM_rxCommandReady = COM_MODULE_COMMAND_BUSY;
    COM_txCommandReady = COM_MODULE_COMMAND_BUSY;
}

void COM_ExecuteCommands(void)
{
    if (COM_MODULE_COMMAND_READY == COM_rxCommandReady)
    {
        PLM_SetReceiveCommand(COM_CurrentRawRXCommandSet);
        /* reset flag to read and execute command in the next cycle */
        COM_rxCommandReady = COM_MODULE_COMMAND_BUSY;
    }

    if (COM_MODULE_COMMAND_READY == COM_txCommandReady)
    {
        HAL_UART_Transmit_IT(&huart3, COM_TxBuffer, COM_RXTX_BUFFER_SIZE);
        /* reset flag to read and execute command in the next cycle */
        COM_txCommandReady = COM_MODULE_COMMAND_BUSY;
    }
}

void COM_SetTransmitCommand(uint8_t *command)
{
    unsigned int i = 0u;

    /* just copy command to module internal buffer */
    for (; i < COM_RXTX_BUFFER_SIZE; i++)
    {
        COM_TxBuffer[i] = command[i];
    }

    /* exit... */
}

void COM_ValidateCommands(void)
{
    unsigned int i = 0u;

    if (HAL_UART_STATE_READY == HAL_UART_GetState(&huart3))
    {
        /* set internal command read flag */
        COM_rxCommandReady = COM_MODULE_COMMAND_READY;

        /* copy current command set to internal buffer,
         * -> latest command is always available in this state variable
         * -> buffered access, so rx buffer is ready to receive next sequence */
        for (; i < COM_RXTX_BUFFER_SIZE; i++)
        {
            COM_CurrentRawRXCommandSet[i] = COM_RxBuffer[i];
        }

        /* ready to receive the next 10 byte sequence */
        HAL_UART_Receive_IT(&huart3, COM_RxBuffer, COM_RXTX_BUFFER_SIZE);
    }
}
