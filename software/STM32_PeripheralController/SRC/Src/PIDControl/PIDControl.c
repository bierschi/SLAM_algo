/*
 * PIDControl.c
 *
 *  Created on: 26.05.2016
 *      Author: stefan
 */

#include "PIDControl.h"

void PIDC_calculateOutput(PIDC_ControllerHandleType *ctrlHnd)
{
    PIDC_ControllerCalculationType errorStep = PIDC_CONTROLLER_CALCULATION_TYPE_ZERO;
    PIDC_ControllerCalculationType targetVel = PIDC_CONTROLLER_CALCULATION_TYPE_ZERO;
    PIDC_ControllerCalculationType errorVel = PIDC_CONTROLLER_CALCULATION_TYPE_ZERO;
    PIDC_ControllerCalculationType tempOutput = PIDC_CONTROLLER_CALCULATION_TYPE_ZERO;
    PIDC_ControllerOutputBaseType resultOutput = PIDC_CONTROLLER_OUTPUT_TYPE_ZERO;
    PIDC_ControllerCalculationType lastEsumStep = PIDC_CONTROLLER_CALCULATION_TYPE_ZERO;
    PIDC_ControllerCalculationType controlValueDistance = PIDC_CONTROLLER_CALCULATION_TYPE_ZERO;
    PIDC_ControllerCalculationType controlValueSpeed = PIDC_CONTROLLER_CALCULATION_TYPE_ZERO;

    if (((*ctrlHnd).esumStep) > PIDC_CONTROLLER_CONSTANT_WINDUP_MAX)
    {
        ((*ctrlHnd).esumStep) = PIDC_CONTROLLER_CONSTANT_WINDUP_MAX;
    }
    else if (((*ctrlHnd).esumStep) < -PIDC_CONTROLLER_CONSTANT_WINDUP_MAX)
    {
        ((*ctrlHnd).esumStep) = -PIDC_CONTROLLER_CONSTANT_WINDUP_MAX;
    }

    /*==================================*/
    /* calculate position error */
    errorStep = (PIDC_ControllerCalculationType) (((*ctrlHnd).stepTargetInput)
            - ((*ctrlHnd).stepMeasInput));

    /* check if target destination was reached and controller integral part can be cleared */
    if (((((*ctrlHnd).stepTargetInput) - ((*ctrlHnd).stepMeasInput))
            < PIDC_CONTROLLER_CONSTANT_ERROR_FLOOR)
            && ((((*ctrlHnd).stepTargetInput) - ((*ctrlHnd).stepMeasInput))
                    > -PIDC_CONTROLLER_CONSTANT_ERROR_FLOOR))
    {
        ((*ctrlHnd).esumStep) = PIDC_CONTROLLER_CALCULATION_TYPE_ZERO;
    }

    lastEsumStep = ((*ctrlHnd).esumStep);

    /* only sum up error values if error value is smaller than threshold value */
    if ((errorStep < PIDC_CONTROLLER_CONSTANT_KI_STEP_THRESHOLD)
            && (errorStep > -PIDC_CONTROLLER_CONSTANT_KI_STEP_THRESHOLD))
    {
        ((*ctrlHnd).esumStep) += errorStep;

        //target_v = (target_velocity_x * e) / KI_ESUM_THRESHOLD;
        ((*ctrlHnd).esumVel) = PIDC_CONTROLLER_CALCULATION_TYPE_ZERO;

        controlValueDistance = (((*ctrlHnd).constantKpStep) * errorStep)
                + (((*ctrlHnd).constantKiStep) * ((*ctrlHnd).esumStep))
                + (((*ctrlHnd).constantKdStep) * (errorStep - ((*ctrlHnd).lastStepError)));
    }
    else
    {
        if (errorStep < PIDC_CONTROLLER_CALCULATION_TYPE_ZERO)
        {
            targetVel = -1.0f * ((PIDC_ControllerCalculationType) ((*ctrlHnd).velTargetInput));
        }
        else
        {
            targetVel = (PIDC_ControllerCalculationType) ((*ctrlHnd).velTargetInput);
        }

        /* calculate velocity error */
        errorVel = targetVel - (PIDC_ControllerCalculationType) ((*ctrlHnd).velMeasInput);
        ((*ctrlHnd).esumVel) += errorVel;
        ((*ctrlHnd).esumStep) = PIDC_CONTROLLER_CALCULATION_TYPE_ZERO;

        controlValueSpeed = (((*ctrlHnd).constantKpVel) * errorVel)
                + (((*ctrlHnd).constantKiVel) * ((*ctrlHnd).esumVel));
    }

    /*==================================*/
    /* PI controller calculation */
    tempOutput = controlValueSpeed + controlValueDistance;

    /*==================================*/
    /* determine direction of movement */
    if (tempOutput < 0.0f)
    {
        (*ctrlHnd).controllerDirection = PIDC_CONTROLLER_DIRECTION_RWD;
        tempOutput = tempOutput * (-1.0f);
    }
    else
    {
        (*ctrlHnd).controllerDirection = PIDC_CONTROLLER_DIRECTION_FWD;
    }
    /*==================================*/

    /* set bounds of y value */
    if (tempOutput > PIDC_CONTROLLER_CONSTANT_MAX_OUTPUT)
    {
        tempOutput = PIDC_CONTROLLER_CONSTANT_MAX_OUTPUT;
        // freeze integral part if output value is out of bounds
        ((*ctrlHnd).esumStep) = lastEsumStep;
    }
    /*=======================*/

    /* cast y output to uint8_t value */
    resultOutput = (PIDC_ControllerOutputBaseType) (tempOutput);

    (*ctrlHnd).controllerOutput = resultOutput;
}

void PIDC_resetStates(PIDC_ControllerHandleType *controllerHandle)
{
    (*controllerHandle).esumStep = PIDC_CONTROLLER_CALCULATION_TYPE_ZERO;
    (*controllerHandle).esumVel = PIDC_CONTROLLER_CALCULATION_TYPE_ZERO;
    (*controllerHandle).controllerOutput = PIDC_CONTROLLER_OUTPUT_TYPE_ZERO;
    (*controllerHandle).controllerDirection = PIDC_CONTROLLER_DIRECTION_FWD;
    (*controllerHandle).lastStepError = PIDC_CONTROLLER_CALCULATION_TYPE_ZERO;
    (*controllerHandle).stepMeasInput = PIDC_CONTROLLER_INPUT_TYPE_ZERO;
    (*controllerHandle).velMeasInput = PIDC_CONTROLLER_INPUT_TYPE_ZERO;
    (*controllerHandle).stepTargetInput = PIDC_CONTROLLER_INPUT_TYPE_ZERO;
    (*controllerHandle).velTargetInput = PIDC_CONTROLLER_INPUT_TYPE_ZERO;
}

uint8_t PIDC_CheckControlQuality(PIDC_ControllerHandleType *controllerHandle)
{
    uint8_t result = 0u;

    if((*controllerHandle).controlQuality > PIDC_CONTROLLER_CONSTANT_MAX_CONTROL_QUALITY)
    {
        /* return true if control quality has been reached */
        result = 1u;
    }

    /* check if control quality has been reached this cycle */
    if((*controllerHandle).controllerOutput == PIDC_CONTROLLER_OUTPUT_TYPE_ZERO)
    {
        /* check if control quality has reached maximum value this cycle */
        if((*controllerHandle).controlQuality <= PIDC_CONTROLLER_CONSTANT_MAX_CONTROL_QUALITY)
        {
            (*controllerHandle).controlQuality += 1.0f;
        }
    }
    else
    {
        /* reset control quality to zero if it was not reached in one cycle */
        (*controllerHandle).controlQuality = PIDC_CONTROLLER_CALCULATION_TYPE_ZERO;
        result = 0u;
    }

    return result;
}
