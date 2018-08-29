/*
 * PIDControl.h
 *
 *  Created on: 26.05.2016
 *      Author: stefan
 */

#ifndef SRC_PIDCONTROL_H_
#define SRC_PIDCONTROL_H_

#include "stdint.h"

#define PIDC_CONTROLLER_CONSTANT_MAX_OUTPUT 999.0f

#define PIDC_CONTROLLER_CONSTANT_KP_STEP 0.9f
#define PIDC_CONTROLLER_CONSTANT_KI_STEP 0.4f
#define PIDC_CONTROLLER_CONSTANT_KD_STEP 0.0f

#define PIDC_CONTROLLER_CONSTANT_KP_VEL 15.0f
#define PIDC_CONTROLLER_CONSTANT_KI_VEL 0.9f

#define PIDC_CONTROLLER_CONSTANT_KI_STEP_THRESHOLD  100.0f
#define PIDC_CONTROLLER_CONSTANT_ERROR_FLOOR        2.0f

#define PIDC_CONTROLLER_CONSTANT_WINDUP_MAX (PIDC_CONTROLLER_CONSTANT_MAX_OUTPUT / PIDC_CONTROLLER_CONSTANT_KI_STEP)

#define PIDC_CONTROLLER_DIRECTION_FWD 0u
#define PIDC_CONTROLLER_DIRECTION_RWD 1u

#define PIDC_CONTROLLER_CALCULATION_TYPE_ZERO  0.0f
#define PIDC_CONTROLLER_OUTPUT_TYPE_ZERO       0u
#define PIDC_CONTROLLER_INPUT_TYPE_ZERO        0

#define PIDC_CONTROLLER_CONSTANT_MAX_CONTROL_QUALITY    5.0f

typedef float PIDC_ControllerCalculationType;

typedef int32_t PIDC_ControllerInputBaseType;

typedef uint16_t PIDC_ControllerOutputBaseType;

typedef uint8_t PIDC_ControllerDirectionType;

typedef struct PIDC_ControllerHandle
{
    PIDC_ControllerCalculationType constantKiStep;
    PIDC_ControllerCalculationType constantKpStep;
    PIDC_ControllerCalculationType constantKdStep;
    PIDC_ControllerCalculationType constantKiVel;
    PIDC_ControllerCalculationType constantKpVel;
    PIDC_ControllerInputBaseType stepMeasInput;
    PIDC_ControllerCalculationType velMeasInput;
    PIDC_ControllerInputBaseType stepTargetInput;
    PIDC_ControllerCalculationType velTargetInput;
    PIDC_ControllerCalculationType esumStep;
    PIDC_ControllerCalculationType esumVel;
    PIDC_ControllerCalculationType lastStepError;
    PIDC_ControllerOutputBaseType controllerOutput;
    PIDC_ControllerDirectionType controllerDirection;
    PIDC_ControllerCalculationType controlQuality;
} PIDC_ControllerHandleType;

extern void PIDC_calculateOutput(PIDC_ControllerHandleType *controllerHandle);

extern void PIDC_resetStates(PIDC_ControllerHandleType *controllerHandle);

extern uint8_t PIDC_CheckControlQuality(PIDC_ControllerHandleType *controllerHandle);

#endif /* SRC_PIDCONTROL_H_ */
