#include "Odometry.h"
#include "ComModule.h"

//#define DEBUG_ODOMETRY

#ifdef DEBUG_ODOMETRY
#include <stdio.h>
#include <string.h>

char bufferString[200] = {0u};
#endif


// distance measurement variables:
OdometryDistanceType CurrentPosition_Y;
OdometryDistanceType CurrentPosition_X;

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

void ODO_Init(void)
{
    CurrentPosition_Y = 0.0f;
    CurrentPosition_X = 0.0f;

    CurrentVelocity_X = 0.0f;
    CurrentVelocity_Y = 0.0f;
    CurrentVelocity = 0.0f;
}

void ODO_PropagateOdometry(void)
{
    LateralAccelValuePhysType x_accel, y_accel, z_accel;
    AngularAccelValuePhysType xr_accel, yr_accel, zr_accel;
    float velocityDecay = 0.02f;

    MPU_GetPhysLateralAccelerations(&x_accel, &y_accel, &z_accel);
    MPU_GetPhysAngularAccelerations(&xr_accel, &yr_accel, &zr_accel);

    // enter Sample Rate here (v = v0 + a * tSA)!!!
    CurrentVelocity_X += (x_accel * 0.01f);
    CurrentVelocity_Y += (y_accel * 0.01f);
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
    CurrentPosition_X += CurrentVelocity_X * 0.01f;
    CurrentPosition_Y += CurrentVelocity_Y * 0.01f;

    #ifdef DEBUG_ODOMETRY
     //snprintf(bufferString, 199,
     //"xAccel:%.2fm/s2\nyAccel%.2fm/s2\nzAccel%.2fm/s2\nxrAccel%.2fdg/s\nyrAccel%.2fdg/s\nzrAccel%.2fdg/s\n",
     //x_accel, y_accel, z_accel, xr_accel, yr_accel, zr_accel);
    snprintf(bufferString, 199, "Distance X: %.4f , Distance Y: %.4f\n", CurrentPosition_X, CurrentPosition_Y);
    COM_PrintToUART((uint8_t *) bufferString, (uint8_t) strlen(bufferString));
    #endif
}
