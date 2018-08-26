/*
 * ServoControl.h
 *
 *  Created on: 07.06.2016
 *      Author: stefan
 */

#ifndef SRC_SERVOCONTROL_SERVOCONTROL_H_
#define SRC_SERVOCONTROL_SERVOCONTROL_H_

#include "stdint.h"

extern void SCM_Init(void);
extern void SCM_TrackOrientation(int32_t transversalError);
extern void SCM_SetTimerValueForAngle(float angle);

#endif /* SRC_SERVOCONTROL_SERVOCONTROL_H_ */
