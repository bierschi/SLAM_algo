/*
 * Odometry.h
 *
 *  Created on: 01.07.2016
 *      Author: stefan
 */

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "MPUSensor.h"

typedef float OdometryDistanceType; // type for odometry distance measurements
typedef float OdometryVelocityType; // type for odometry velocity measurements
typedef int32_t OdometryDistanceType_mm; // type for odometry distance measurements in mm

extern OdometryVelocityType ODO_GetCurrentVelocityY(void);
extern OdometryDistanceType ODO_GetCurrentPositionY(void);

extern OdometryVelocityType ODO_GetCurrentVelocityX(void);
extern OdometryDistanceType ODO_GetCurrentPositionX(void);

extern OdometryDistanceType_mm ODO_GetCurrentPositionX_mm(void);

extern OdometryDistanceType_mm ODO_GetCurrentPositionY_mm(void);

extern void ODO_Init(void);

extern void ODO_PropagateOdometry(void);

#endif
