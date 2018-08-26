/*
 * Odometry.h
 *
 *  Created on: 01.07.2016
 *      Author: stefan
 */

#ifndef ODOMETRY_H
#define ODOMETRY_H

typedef float OdometryDistanceType; // type for odometry distance measurements

extern int ODO_GetCurrentVelocityY(void);
extern int ODO_GetCurrentPositionY(void);

extern int ODO_GetCurrentVelocityX(void);
extern int ODO_GetCurrentPositionX(void);

#endif