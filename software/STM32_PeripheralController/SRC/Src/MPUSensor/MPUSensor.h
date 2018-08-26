/*
 * MPUSensor.h
 *
 *  Created on: 23.08.2018
 *      Author: stefan
 */

#ifndef MPU_SENSOR_H
#define MPU_SENSOR_H

#include "stdint.h"
#include "PeripheralHandles.h"

#define MPU_LATERAL_ACCEL_CONV_FACTOR ((4.0f*9.81f)/32768.0f)
#define MPU_ANGULAR_ACCEL_CONV_FACTOR (500.0f/32768.0f)
#define MPU_LATERAL_ACCEL_ZERO 0.0f
#define MPU_ANGULAR_ACCEL_ZERO 0.0f

typedef enum 
{
    ACCEL_TYPE_X,
    ACCEL_TYPE_Y,
    ACCEL_TYPE_Z,
    ACCEL_TYPE_RX,
    ACCEL_TYPE_RY,
    ACCEL_TYPE_RZ
} AccelType;

typedef float LateralAccelValuePhysType; /* Physical values from sensor () */
typedef int16_t LateralAccelValueRawType; /* Raw data from sensor registers */
typedef float AngularAccelValuePhysType; /* Physical values from sensor (angular accel.) */
typedef int16_t AngularAccelValueRawType; /* Physical values from sensor (angular accel.) */

extern void MPU_SetPhysLateralAccelerations(void);

extern void MPU_SetPhysAngularAccelerations(void);

extern void MPU_GetPhysLateralAccelerations(LateralAccelValuePhysType * currentXAccel, LateralAccelValuePhysType * currentYAccel, LateralAccelValuePhysType * currentZAccel);

extern void MPU_GetPhysAngularAccelerations(LateralAccelValuePhysType * currentXRAccel, LateralAccelValuePhysType * currentYRAccel, LateralAccelValuePhysType * currentZRAccel);

extern void MPU_Init(void);

extern void MPU_ReadValues(void);

#endif