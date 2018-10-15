/*
 * PlannerModule.c
 *
 *  Created on: 01.07.2016
 *      Author: stefan
 */

#include "PlannerModule.h"
#include "Timing.h"
#include "PIDControl.h"
#include "MotorControl.h"
#include "ServoControl.h"
#include "ComModule.h"
#include "Odometry.h"
#include "SRFSensor.h"

#define PLM_SERVO_ROTATION_WAIT_TIME    2.0f

//#define DEBUG_PLANNER

#ifdef DEBUG_PLANNER
#include <stdio.h>
#include <string.h>

char bufferString[200] = {0u};
#endif

/* local module data types */
typedef struct CoordinatePoint {
	int32_t x;
	int32_t y;
} CoordinatePointType;

/* local module variable instances */
PIDC_ControllerHandleType MainEngine;
uint32_t PLM_CurrentControlState;

CoordinatePointType PLM_CurrentCoordinateTarget;
float PLM_CurrentVelocityTarget;

float PLM_WaitTimer;

void PLM_StartWaitTimer(float time) {
	PLM_WaitTimer = time;
}

uint8_t PLM_IsWaitTimerRunning(void) {
	if (0.0f < PLM_WaitTimer) {
		return 1u;
	} else {
		return 0u;
	}
}

void PLM_PropagateWaitTimer(void) {
	if (0.0f <= PLM_WaitTimer) {
		PLM_WaitTimer -= MAIN_SAMPLE_TIME_S;
	}
}

void PLM_PrepareTransmitMessages(void)
{
	// transmit current orientation over SPI channel (DMA)
	COM_StructTX.CurrentOrientation = ODO_GetCurrentOrientationZR();
	COM_StructTX.CurrentPositionX = ODO_GetCurrentPositionX();
	COM_StructTX.CurrentPositionY = ODO_GetCurrentPositionY();
	COM_StructTX.USDistanceFrontLeft = SRF_GetDistanceFrontLeft();
	COM_StructTX.USDistanceFrontRight = SRF_GetDistanceFrontRight();
	COM_StructTX.USDistanceRear = SRF_GetDistanceRear();
}

void PLM_DetermineSteering(void) {
	if (COM_StructRX.CurrentSteeringMode == COM_STEERING_MODE_AUTO) {
		PLM_CurrentCoordinateTarget.x =
				//(PIDC_ControllerInputBaseType) (COM_Struct.Target_X * 400.0f / 25.4f);
				(PIDC_ControllerInputBaseType) 60.0f;
		PLM_CurrentCoordinateTarget.y =
				//(PIDC_ControllerInputBaseType) (COM_Struct.Target_Y * 400.0f / 25.4f);
				0.0f;
	} else if (COM_StructRX.CurrentSteeringMode == COM_STEERING_MODE_MANUAL) {
		float directionTemp = COM_StructRX.CurrentSteeringDirection;
		if (directionTemp > -90.0f && directionTemp < 90.0f) {
			SCM_SetTimerValueForAngle(directionTemp);
		}

		uint16_t speed = COM_StructRX.CurrentSteeringSpeed;
		uint8_t direction = COM_StructRX.CurrentSteeringDirection;
		if (speed <= 1000 && direction < 3u) {
			MTC_SetMotorSpeed(speed);
			MTC_SetMotorDirection(direction);
		}
	}
}

/* main cycle routines for planner execution */
void PLM_PlannerCycle(void) {
	switch (PLM_CurrentControlState) {
	case PLM_MODULE_STATE_STOP:
		/* immediately stop motors */
		MTC_SetMotorSpeed(0u);

		PLM_CurrentControlState = PLM_MODULE_STATE_WAITING;
		break;
	case PLM_MODULE_STATE_TRANSIT_FWD:

		/* check if target was reached: */
		if (1u == PIDC_CheckControlQuality(&MainEngine)) {
			PLM_CurrentControlState = PLM_MODULE_STATE_STOP;
		}

		break;
	case PLM_MODULE_STATE_TRANSIT_SWD:

		/* check if target was reached: */
		if (1u == PIDC_CheckControlQuality(&MainEngine)) {
			PLM_CurrentControlState = PLM_MODULE_STATE_ROTATE_SCM_FWD;

			/* start wait timer for next transition */
			PLM_StartWaitTimer(PLM_SERVO_ROTATION_WAIT_TIME);
		}

		break;
	case PLM_MODULE_STATE_ROTATE_SCM_FWD:

		if (0u == PLM_IsWaitTimerRunning()) {
			PLM_CurrentControlState = PLM_MODULE_STATE_TRANSIT_FWD;
			MainEngine.controlQuality = 0.0f;
			PIDC_resetStates(&MainEngine);
		} else {
			SCM_SetTimerValueForAngle(0.0f);
		}

		break;
	case PLM_MODULE_STATE_ROTATE_SCM_SWD:

		if (0u == PLM_IsWaitTimerRunning()) {
			PLM_CurrentControlState = PLM_MODULE_STATE_TRANSIT_SWD;
			MainEngine.controlQuality = 0.0f;
			PIDC_resetStates(&MainEngine);
		} else {
			SCM_SetTimerValueForAngle(90.0f);
		}

		break;
	default:
		break;
	}
}

void PLM_ControllerCycle(void) {

	if (PLM_MODULE_STATE_TRANSIT_SWD == PLM_CurrentControlState) {
		MainEngine.velMeasInput = ODO_GetCurrentVelocityY();
		MainEngine.stepMeasInput = ODO_GetCurrentPositionY_mm();
		MainEngine.stepTargetInput = PLM_CurrentCoordinateTarget.y;
		MainEngine.velTargetInput = PLM_CurrentVelocityTarget;
	} else if (PLM_MODULE_STATE_TRANSIT_FWD == PLM_CurrentControlState) {
		MainEngine.velMeasInput = ODO_GetCurrentVelocityX();
		MainEngine.stepMeasInput = ODO_GetCurrentPositionX_mm();
		MainEngine.stepTargetInput = PLM_CurrentCoordinateTarget.x;
		MainEngine.velTargetInput = PLM_CurrentVelocityTarget;
	}

	// prepare communication structure for SPI
	PLM_PrepareTransmitMessages();

	if ((PLM_MODULE_STATE_TRANSIT_SWD == PLM_CurrentControlState)
			|| (PLM_MODULE_STATE_TRANSIT_FWD == PLM_CurrentControlState)) {
		PIDC_calculateOutput(&MainEngine);

		MTC_SetMotorSpeed(MainEngine.controllerOutput);
		MTC_SetMotorDirection(MainEngine.controllerDirection);

		if (PLM_MODULE_STATE_TRANSIT_FWD == PLM_CurrentControlState) {
			/* only track orientation when transiting forwards */
			//SCM_TrackOrientation(PLM_CurrentCoordinateTarget.y - ODO_GetCurrentPositionY());
			SCM_SetTimerValueForAngle((0.0f + ODO_GetCurrentOrientationZR()) + 0.0f);
		}
	} else {
		/* turn off motors immediately */
		MTC_SetMotorSpeed(0u);
	}
}

void PLM_Init(void) {
	PIDC_resetStates(&MainEngine);

	PLM_CurrentControlState = PLM_MODULE_STATE_START;

	MainEngine.constantKdStep = 0.0f;
	MainEngine.constantKiStep = 0.04f;
	MainEngine.constantKpStep = 0.14f;
	MainEngine.constantKiVel = 0.9f;
	MainEngine.constantKpVel = 15.0f;

	MainEngine.stepMeasInput = 0;
	MainEngine.velMeasInput = 0;

	MainEngine.stepTargetInput = 0;
	MainEngine.velTargetInput = 2.0f; /* -> 2cm/s */

	PLM_CurrentCoordinateTarget.x = 0; // steps in mm
	PLM_CurrentCoordinateTarget.y = 0; // steps in mm

	PLM_CurrentVelocityTarget = 3.0f;

	PLM_WaitTimer = -1.0f;

	SCM_SetTimerValueForAngle(0.0f);
}

void PLM_MainCycle(void) {
	PLM_CurrentControlState = PLM_MODULE_STATE_TRANSIT_FWD;

	PLM_DetermineSteering();
	/* do main trajectory planning an control planning */

	if (1u) {
		// PLM_PlannerCycle();

		/* update controllers */
		PLM_ControllerCycle();
	}

	/* propagate local delay timer */
	PLM_PropagateWaitTimer();

#ifdef DEBUG_PLANNER
	snprintf(bufferString, 199, "SteeringMode: %i\n", COM_Struct.CurrentSteeringMode);
	COM_PrintToUART((uint8_t *) bufferString, (uint8_t) strlen(bufferString));
#endif
}
