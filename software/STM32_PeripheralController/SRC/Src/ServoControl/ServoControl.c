/*
 * ServoControl.c
 *
 *  Created on: 07.06.2016
 *      Author: stefan
 */

#include "ServoControl.h"
#include "PeripheralHandles.h"
#include "MotorControl.h"

float SCM_CurrentAngle;

void SCM_SetTimerValueForAngle(float angle)
{
	// init value of timer register:
    uint32_t timerValue = 750u;

    if ((angle <= SCM_SERVO_MAX_ANGLE_POS) && (angle >= SCM_SERVO_MIN_ANGLE_POS))
    {
    	// calculations derived from CPU Frequency, Timer register max, and Servo steering conventions
        timerValue = (uint32_t) ((angle * 4.778f) + 750.0f);
    }

    SCM_CurrentAngle = angle;

    /* write result to timer compare register 1 (TIM2 Channel 1) */
    htim3.Instance->CCR2 = timerValue;
}

void SCM_TrackOrientation(int32_t transversalError)
{
    float e = 0.0f;
    float y = 0.0f;

    e = (float) transversalError;

    if (MTC_GetMotorDirection() == MTC_MOTOR_DIRECTION_FWD)
        y = e * 0.7f;
    if (MTC_GetMotorDirection() == MTC_MOTOR_DIRECTION_RWD)
        y = e * -0.7f;

    /* set bounds of y value */
    if (y > SCM_SERVO_MAX_ANGLE_POS)
    {
        y = SCM_SERVO_MAX_ANGLE_POS;
    }
    if (y < SCM_SERVO_MIN_ANGLE_POS)
    {
        y = SCM_SERVO_MIN_ANGLE_POS;
    }

    SCM_SetTimerValueForAngle(y);
}

void SCM_Init(void)
{
    SCM_CurrentAngle = 0.0f;
}

