/*
 * ComModule.h
 *
 *  Created on: 12.06.2016
 *      Author: stefan
 */

#ifndef SRC_COMMODULE_COMMODULE_H_
#define SRC_COMMODULE_COMMODULE_H_

#include "stdint.h"

#define COM_MODULE_COMMAND_READY            1u
#define COM_MODULE_COMMAND_BUSY             0u

#define COM_RXTX_BUFFER_SIZE                10


extern void COM_Init(void);
extern void COM_ValidateCommands(void);
extern void COM_ExecuteCommands(void);
extern void COM_SetTransmitCommand(uint8_t *command);
extern void COM_PrintToUART(uint8_t *data, uint16_t size);

#endif /* SRC_COMMODULE_COMMODULE_H_ */
