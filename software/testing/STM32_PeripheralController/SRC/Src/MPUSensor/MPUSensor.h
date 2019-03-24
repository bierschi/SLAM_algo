/*
 * MPUSensor.h
 *
 *  Created on: 23.08.2018
 *      Author: stefan
 */

#ifndef MPU_SENSOR_H
#define MPU_SENSOR_H

#include <stdint.h>
#include "PeripheralHandles.h"

#define MPU_LATERAL_ACCEL_CONV_FACTOR ((4.0f*9.81f)/32768.0f)
#define MPU_ANGULAR_VEL_CONV_FACTOR (500.0f/32768.0f)
#define MPU_ANGULAR_VEL_OFFSET 1.15f
#define MPU_LATERAL_ACCEL_ZERO 0.0f
#define MPU_ANGULAR_VEL_ZERO 0.0f

typedef float LateralAccelValuePhysType; /* Physical values from sensor () */
typedef int16_t LateralAccelValueRawType; /* Raw data from sensor registers */
typedef float AngularVelValuePhysType; /* Physical values from sensor (angular accel.) */
typedef int16_t AngularVelValueRawType; /* Physical values from sensor (angular accel.) */

extern void MPU_SetPhysLateralAccelerations(void);

extern void MPU_SetPhysAngularAccelerations(void);

extern void MPU_GetPhysLateralAccelerations(LateralAccelValuePhysType * currentXAccel, LateralAccelValuePhysType * currentYAccel, LateralAccelValuePhysType * currentZAccel);

extern void MPU_GetPhysAngularVelocity(AngularVelValuePhysType * currentXRAccel, AngularVelValuePhysType * currentYRAccel, AngularVelValuePhysType * currentZRAccel);

extern void MPU_Init(void);

extern void MPU_ReadValues(void);

#endif
