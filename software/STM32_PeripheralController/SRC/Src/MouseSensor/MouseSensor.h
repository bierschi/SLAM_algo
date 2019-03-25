/*
 * MouseSensor.h
 *
 *  Created on: 03.06.2016
 *      Author: stefan
 */

#ifndef MOUSE_SENSOR_H
#define MOUSE_SENSOR_H

#include <stdint.h>

#define MSC_FACTOR_CPI_TO_CM (2.54f/400.0f)
#define MSC_USED_SAMPLE_TIME 10.0e-3f

#define MSC_DISTANCE_FROM_TURNPOINT  0.065f

/* currently acquired x and y deviations from mouse sensor */
extern int32_t MSC_CurrentYDelta;
extern int32_t MSC_CurrentXDelta;

/* global x and y positions */
extern int32_t MSC_GetCurrentPositionX(void);
extern int32_t MSC_GetCurrentPositionY(void);

extern float MSC_GetCurrentVelocityX(void);
extern float MSC_GetCurrentVelocityY(void);

extern void MSC_SensorReset(void);

extern unsigned char MSC_SensorReadRegister(unsigned char address);

extern void MSC_SensorWriteRegister(unsigned char address, unsigned char data);

extern void MSC_SensorAcquirePosition(void);

#endif
