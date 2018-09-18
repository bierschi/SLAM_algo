/*
 * MotorControl.h
 *
 *  Created on: 04.06.2016
 *      Author: stefan
 */

#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_

#include <stdint.h>
#include "PIDControl.h"

#define MTC_MOTOR_DIRECTION_FWD     PIDC_CONTROLLER_DIRECTION_FWD
#define MTC_MOTOR_DIRECTION_RWD     PIDC_CONTROLLER_DIRECTION_RWD

#define MTC_CONTROL_REGISTER    (htim2.Instance)

extern void MTC_Init(void);
extern void MTC_SetMotorDirection(uint8_t direction);
extern void MTC_SetMotorSpeed(uint16_t speed);
extern void MTC_ShutdownMotor(void);
extern uint8_t MTC_GetMotorDirection(void);

#endif /* MOTORCONTROL_H_ */
