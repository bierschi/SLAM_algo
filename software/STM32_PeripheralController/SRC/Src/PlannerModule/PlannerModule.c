/*
 * PlannerModule.c
 *
 *  Created on: 01.07.2016
 *      Author: stefan
 */

#include "PlannerModule.h"
#include "PIDControl.h"
#include "MotorControl.h"
#include "ServoControl.h"
#include "ComModule.h"
#include "Odometry.h"

#ifdef DEBUG_PLANNER
#include <stdio.h>
#include <string.h>

char bufferString[200] = {0u};
#endif

/* local module data types */
typedef struct CoordinatePoint
{
    int32_t x;
    int32_t y;
} CoordinatePointType;

/* local module variable instances */
PIDC_ControllerHandleType MainEngine;

uint8_t PLM_CurrentReceiveCommand[COM_RXTX_BUFFER_SIZE];
uint8_t PLM_CurrentTransmitCommand[COM_RXTX_BUFFER_SIZE];
uint32_t PLM_CurrentControlState;

CoordinatePointType PLM_CurrentCoordinateTarget;
float PLM_CurrentVelocityTarget;

float PLM_WaitTimer;

/* internal setter and getter routines */

int32_t PLM_GetInt32FromFourCharacters(uint8_t *characters)
{
    int32_t temp = 0;
    uint32_t tempU = 0u;
    int32_t * tempUPointer = (int32_t *) &tempU;

    /* convert 4 byte unsigned char to int32 representation */
    tempU = ((uint32_t) characters[3] << 24) | ((uint32_t) characters[2] << 16) | ((uint32_t) characters[1] << 8)
            | ((uint32_t) characters[0]);

    temp = (*tempUPointer);

    return temp;
}

/* setter routines for Planner module */
void PLM_SetTransmitCommand(uint8_t *command)
{
    /* currently empty function, as it transmits nothing to the master board */
}

void PLM_SetReceiveCommand(uint8_t *command)
{
    unsigned int i = 0u;

    /* just copy command to module internal buffer */
    for (; i < COM_RXTX_BUFFER_SIZE; i++)
    {
        PLM_CurrentReceiveCommand[i] = command[i];
    }

    /* exit... */
}

void PLM_StartWaitTimer(float time)
{
    PLM_WaitTimer = time;
}

uint8_t PLM_IsWaitTimerRunning(void)
{
    if (0.0f < PLM_WaitTimer)
    {
        return 1u;
    }
    else
    {
        return 0u;
    }
}

void PLM_PropagateWaitTimer(void)
{
    if (0.0f <= PLM_WaitTimer)
    {
        PLM_WaitTimer -= 0.01f;
    }
}

void PLM_PrepareTransmitMessage(uint8_t message)
{
    unsigned int i = 0u;

    switch (message)
    {
    case PLM_MODULE_TRANSMIT_MSG_READY:

        PLM_CurrentTransmitCommand[0] = PLM_MODULE_TRANSMIT_MSG_READY;

        for (i = 0u; i < COM_RXTX_BUFFER_SIZE; i++)
        {
            PLM_CurrentTransmitCommand[i] = 0u;
        }

        break;
    case PLM_MODULE_TRANSMIT_MSG_ERROR:

        PLM_CurrentTransmitCommand[0] = PLM_MODULE_TRANSMIT_MSG_ERROR;

        for (i = 0u; i < COM_RXTX_BUFFER_SIZE; i++)
        {
            PLM_CurrentTransmitCommand[i] = 0u;
        }

        break;
    }
}

/* main cycle routines for planner execution */
void PLM_PlannerCycle(void)
{
    switch (PLM_CurrentControlState)
    {
    case PLM_MODULE_STATE_STOP:
        /* immediately stop motors */
        MTC_SetMotorSpeed(0u);

        PLM_CurrentControlState = PLM_MODULE_STATE_WAITING;
        break;
    case PLM_MODULE_STATE_TRANSIT_FWD:

        /* check if target was reached: */
        if (1u == PIDC_CheckControlQuality(&MainEngine))
        {
            PLM_CurrentControlState = PLM_MODULE_STATE_STOP;
        }

        break;
    case PLM_MODULE_STATE_TRANSIT_SWD:

        /* check if target was reached: */
        if (1u == PIDC_CheckControlQuality(&MainEngine))
        {
            PLM_CurrentControlState = PLM_MODULE_STATE_ROTATE_SCM_FWD;

            /* start wait timer for next transition */
            PLM_StartWaitTimer(2.0f);
        }

        break;
    case PLM_MODULE_STATE_ROTATE_SCM_FWD:

        if (0u == PLM_IsWaitTimerRunning())
        {
            PLM_CurrentControlState = PLM_MODULE_STATE_TRANSIT_FWD;
            MainEngine.controlQuality = 0.0f;
            PIDC_resetStates(&MainEngine);
        }
        else
        {
            SCM_SetTimerValueForAngle(0.0f);
        }

        break;
    case PLM_MODULE_STATE_ROTATE_SCM_SWD:

        if (0u == PLM_IsWaitTimerRunning())
        {
            PLM_CurrentControlState = PLM_MODULE_STATE_TRANSIT_SWD;
            MainEngine.controlQuality = 0.0f;
            PIDC_resetStates(&MainEngine);
        }
        else
        {
            SCM_SetTimerValueForAngle(90.0f);
        }

        break;
    default:
        break;
    }
}

void PLM_CommandTransmitCycle(void)
{
    if(PLM_CurrentTransmitCommand[0] != 0u)
    {
        COM_SetTransmitCommand(&PLM_CurrentTransmitCommand[0]);
    }

    PLM_CurrentTransmitCommand[0] = 0u;
}

void PLM_CommandReceiveCycle(void)
{
    int32_t tempIntX = 0;
    int32_t tempIntY = 0;

    float tempFloat = 0.0f;
    int32_t tempInt32 = 0;

    /* select command from first character of buffer */
    switch (PLM_CurrentReceiveCommand[0])
    {
    case PLM_MODULE_COMMAND_SET_POSITION: /* set position coordinates to travel next */
        /*parse string information to float */
        tempIntX = PLM_GetInt32FromFourCharacters(&PLM_CurrentReceiveCommand[1]);
        tempIntY = PLM_GetInt32FromFourCharacters(&PLM_CurrentReceiveCommand[5]);

        /* set new coordinates */
        PLM_CurrentCoordinateTarget.x = tempIntX;
        PLM_CurrentCoordinateTarget.y = tempIntY;

        /* restart control, next control state is to travel sidewards */
        PLM_CurrentControlState = PLM_MODULE_STATE_ROTATE_SCM_SWD;
        MainEngine.controlQuality = 0.0f;

        PLM_StartWaitTimer(2.0f);

        break;
    case PLM_MODULE_COMMAND_SET_VELOCITY: /* set position coordinates to travel next */
        /*parse string information to float */
        tempInt32 = PLM_GetInt32FromFourCharacters(&PLM_CurrentReceiveCommand[1]);

        tempFloat = (float) tempInt32 * PLM_MODULE_INT32_TO_FLT_SCALING;

        /* set new velocity */
        PLM_CurrentVelocityTarget = tempFloat;

        break;
    case PLM_MODULE_COMMAND_SET_ORIENTATION: /* set current orientation */
        break;
    case PLM_MODULE_COMMAND_START_CONTROL:
        PLM_CurrentControlState = PLM_MODULE_STATE_ROTATE_SCM_SWD;
        PLM_StartWaitTimer(3.0f);
        #ifdef DEBUG_PLANNER
        snprintf(bufferString, 199, "SetServoDirection!\n");
        COM_PrintToUART((uint8_t *) bufferString, (uint8_t) strlen(bufferString));
        #endif
        break;
    case PLM_MODULE_COMMAND_STOP_CONTROL:
        PLM_CurrentControlState = PLM_MODULE_STATE_STOP;
        break;
    default:
        break;
    }

    /* invalidate the current command, as it should be only processed once */
    PLM_CurrentReceiveCommand[0] = PLM_MODULE_COMMAND_SET_INVALID;
}

void PLM_ControllerCycle(void)
{
    if (PLM_MODULE_STATE_TRANSIT_SWD == PLM_CurrentControlState)
    {
        MainEngine.velMeasInput = ODO_GetCurrentVelocityY();
        MainEngine.stepMeasInput = 15748 * ODO_GetCurrentPositionY();
        MainEngine.stepTargetInput = PLM_CurrentCoordinateTarget.y;
        MainEngine.velTargetInput = PLM_CurrentVelocityTarget;
    }
    else if (PLM_MODULE_STATE_TRANSIT_FWD == PLM_CurrentControlState)
    {
        MainEngine.velMeasInput = ODO_GetCurrentVelocityX();
        MainEngine.stepMeasInput = 15748 * ODO_GetCurrentPositionX();
        MainEngine.stepTargetInput = PLM_CurrentCoordinateTarget.x;
        MainEngine.velTargetInput = PLM_CurrentVelocityTarget;
    }

    if ((PLM_MODULE_STATE_TRANSIT_SWD == PLM_CurrentControlState)
            || (PLM_MODULE_STATE_TRANSIT_FWD == PLM_CurrentControlState))
    {
        PIDC_calculateOutput(&MainEngine);

        MTC_SetMotorSpeed(MainEngine.controllerOutput);
        MTC_SetMotorDirection(MainEngine.controllerDirection);

        if (PLM_MODULE_STATE_TRANSIT_FWD == PLM_CurrentControlState)
        {
            /* only track orientation when transiting forwards */
            SCM_TrackOrientation(PLM_CurrentCoordinateTarget.y - ODO_GetCurrentPositionY());
        }
    }
    else
    {
        /* turn off motors immediately */
        MTC_SetMotorSpeed(0u);
    }
}

void PLM_Init(void)
{
    PIDC_resetStates(&MainEngine);

    PLM_CurrentControlState = PLM_MODULE_STATE_START;
    PLM_CurrentReceiveCommand[0] = PLM_MODULE_COMMAND_START_CONTROL;

    MainEngine.constantKdStep = 0.0f;
    MainEngine.constantKiStep = 0.04f;
    MainEngine.constantKpStep = 0.14f;
    MainEngine.constantKiVel = 0.9f;
    MainEngine.constantKpVel = 15.0f;

    MainEngine.stepMeasInput = 0;
    MainEngine.velMeasInput = 0;

    MainEngine.stepTargetInput = 200;
    MainEngine.velTargetInput = 2.0f; /* -> 2cm/s */

    PLM_CurrentCoordinateTarget.x = -200;
    PLM_CurrentCoordinateTarget.y = -200;

    PLM_CurrentVelocityTarget = 5.0f;

    PLM_WaitTimer = -1.0f;

    SCM_SetTimerValueForAngle(0.0f);
}

void PLM_MainCycle(void)
{
    /* determine receive commands */
    PLM_CommandReceiveCycle();
    /* determine transmit commands */
    PLM_CommandTransmitCycle();
    /* do main trajectory planning an control planning */
    PLM_PlannerCycle();
    /* update controllers */
    PLM_ControllerCycle();
    /* propagate local delay timer */
    PLM_PropagateWaitTimer();
}
