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
    float temp = 0.0f;
    uint32_t timerValue = 640u;

    if ((angle <= 90.0f) && (angle >= -90.0f))
    {
        temp = (angle * 7.1111f) + 900.0f;
    }

    if ((temp >= 320.0f) && (temp <= 1600.0f))
    {
        timerValue = (uint32_t) temp;
    }

    SCM_CurrentAngle = angle;

    /* write result to timer compare register 1 (TIM2 Channel 1) */
    htim2.Instance->CCR1 = timerValue;
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
    if (y > 90.0f)
    {
        y = 90.0f;
    }
    if (y < -90.0f)
    {
        y = -90.0f;
    }

    SCM_SetTimerValueForAngle(y);
}

void SCM_Init(void)
{
    SCM_CurrentAngle = 0.0f;
}

