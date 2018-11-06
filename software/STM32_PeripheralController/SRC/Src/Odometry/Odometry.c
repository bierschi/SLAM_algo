#include "Odometry.h"
#include "ComModule.h"
#include "MotorControl.h"
#include "Timing.h"

#define DEBUG_ODOMETRY

#ifdef DEBUG_ODOMETRY
#include <stdio.h>
#include <string.h>

char bufferString[200] = {0u};
#endif

// threshold of m
#define ODO_INCREMENT_MOTOR_THRESHOLD	400u


// distance measurement variables:
OdometryDistanceType CurrentPosition_Y;
OdometryDistanceType CurrentPosition_X;

// Current orientation in degree (z - axis)
OdometryAngularType CurrentOrientation_ZR;
OdometryAngularType CurrentOrientation_XR;
OdometryAngularType CurrentOrientation_YR;

OdometryVelocityType CurrentVelocity_Y;
OdometryVelocityType CurrentVelocity_X;

OdometryVelocityType CurrentVelocity;

OdometryVelocityType ODO_GetCurrentVelocityY(void)
{
    return CurrentVelocity_Y;
}

OdometryDistanceType ODO_GetCurrentPositionY(void)
{
    return CurrentPosition_Y;
}

OdometryVelocityType ODO_GetCurrentVelocityX(void)
{
    return CurrentVelocity_X;
}

OdometryDistanceType ODO_GetCurrentPositionX(void)
{
    return CurrentPosition_X;
}

OdometryDistanceType_mm ODO_GetCurrentPositionX_mm(void)
{
	return (OdometryDistanceType_mm) (CurrentPosition_X * 1000.0f);
}

OdometryDistanceType_mm ODO_GetCurrentPositionY_mm(void)
{
	return (OdometryDistanceType_mm) (CurrentPosition_Y * 1000.0f);
}

OdometryAngularType ODO_GetCurrentOrientationXR(void)
{
	return CurrentOrientation_XR;
}

OdometryAngularType ODO_GetCurrentOrientationYR(void)
{
	return CurrentOrientation_YR;
}

OdometryAngularType ODO_GetCurrentOrientationZR(void)
{
	return CurrentOrientation_ZR;
}

void ODO_Init(void)
{
    CurrentPosition_Y = 0.0f;
    CurrentPosition_X = 0.0f;

    CurrentVelocity_X = 0.0f;
    CurrentVelocity_Y = 0.0f;
    CurrentVelocity = 0.0f;
}

// main loop to determine current position and velocity
void ODO_PropagateOdometry(void)
{
    LateralAccelValuePhysType x_accel, y_accel, z_accel;
    AngularVelValuePhysType xr_vel, yr_vel, zr_vel;
    float velocityDecay = 0.001f;

    MPU_GetPhysLateralAccelerations(&x_accel, &y_accel, &z_accel);
    MPU_GetPhysAngularVelocity(&xr_vel, &yr_vel, &zr_vel);

    // enter Sample Rate here (v = v0 + a * tSA)!!!
//	if (MTC_GetMotorSpeed() > ODO_INCREMENT_MOTOR_THRESHOLD) {
		CurrentVelocity_X += (x_accel * MAIN_SAMPLE_TIME_S);
		CurrentVelocity_Y += (y_accel * MAIN_SAMPLE_TIME_S);
//	}
    if(CurrentVelocity_X > 0.0f)
    {
        CurrentVelocity_X -= velocityDecay;
    }
    else if(CurrentVelocity_X < 0.0f)
    {
        CurrentVelocity_X += velocityDecay;
    }

    if(CurrentVelocity_Y > 0.0f)
    {
        CurrentVelocity_Y -= velocityDecay;
    }
    else if(CurrentVelocity_Y < 0.0f)
    {
        CurrentVelocity_Y += velocityDecay;
    }

    // x = x0 + v * tSA
    CurrentPosition_X += CurrentVelocity_X * MAIN_SAMPLE_TIME_S;
    CurrentPosition_Y += CurrentVelocity_Y * MAIN_SAMPLE_TIME_S;

    CurrentOrientation_ZR += zr_vel * MAIN_SAMPLE_TIME_S;
    CurrentOrientation_XR += xr_vel * MAIN_SAMPLE_TIME_S;
    CurrentOrientation_YR += yr_vel * MAIN_SAMPLE_TIME_S;

    #ifdef DEBUG_ODOMETRY
//     snprintf(bufferString, 199,
//     "xAccel:%.2fm/s2\nyAccel%.2fm/s2\nzAccel%.2fm/s2\nxrAccel%.2fdg/s\nyrAccel%.2fdg/s\nzrAccel%.2fdg/s\n",
//     x_accel, y_accel, z_accel, xr_vel, yr_vel, zr_vel);
//    snprintf(bufferString, 199, "Distance X: %.4f , Distance Y: %.4f\n", CurrentPosition_X, CurrentPosition_Y);
//    COM_PrintToUART((uint8_t *) bufferString, (uint8_t) strlen(bufferString));
//    snprintf(bufferString, 199, "Xrot: %.2f\nYrot: %.2f\nZrot: %.2f\n", CurrentOrientation_XR, CurrentOrientation_YR, CurrentOrientation_ZR);
//    COM_PrintToUART((uint8_t *) bufferString, (uint8_t) strlen(bufferString));
    #endif
}
