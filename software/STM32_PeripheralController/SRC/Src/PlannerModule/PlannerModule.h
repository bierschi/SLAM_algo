/*
 * PlannerModule.h
 *
 *  Created on: 01.07.2016
 *      Author: stefan
 */

#ifndef PLANNERMODULE_PLANNERMODULE_H_
#define PLANNERMODULE_PLANNERMODULE_H_

#include "stdint.h"

#define PLM_MODULE_STATE_START              1u
#define PLM_MODULE_STATE_STOP               2u
#define PLM_MODULE_STATE_TRANSIT_FWD        3u
#define PLM_MODULE_STATE_TRANSIT_RWD        4u
#define PLM_MODULE_STATE_TRANSIT_SWD        5u
#define PLM_MODULE_STATE_ROTATE_SCM_FWD     6u
#define PLM_MODULE_STATE_ROTATE_SCM_SWD     7u
#define PLM_MODULE_STATE_WAITING            8u

#define PLM_MODULE_TRANSMIT_MSG_READY       1u
#define PLM_MODULE_TRANSMIT_MSG_ERROR       255u

#define PLM_MODULE_INT32_TO_FLT_SCALING     (0.001f)


extern void PLM_SetTransmitCommand(uint8_t *command);

extern void PLM_SetReceiveCommand(uint8_t *command);

extern void PLM_PlannerCycle(void);

extern void PLM_CommandTransmitCycle(void);

extern void PLM_CommandReceiveCycle(void);

extern void PLM_ControllerCycle(void);

extern void PLM_MainCycle(void);

extern void PLM_Init(void);

#endif /* PLANNERMODULE_PLANNERMODULE_H_ */
