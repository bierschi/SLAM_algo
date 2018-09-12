/*
 * MotorControl.c
 *
 *  Created on: 04.06.2016
 *      Author: stefan
 */

#include "MotorControl.h"
#include "PeripheralHandles.h"  /* get access to some global peripherals and data handles */
#include "stm32f3xx_hal.h"      /* get access to motor control gpios */
#include "mxconstants.h"        /* get access to the gpio pin defines */

uint8_t MTC_CurrentMotorDirection;

void MTC_ShiftMotorDirToOutput(uint8_t direction)
{
    uint8_t data = 0u;

    if(direction == MTC_MOTOR_DIRECTION_FWD)
    {
        GPIOB->BSRR = GPIO_BSRR_BS_5;
    }
    else if(MTC_MOTOR_DIRECTION_RWD)
    {
        GPIOB->BSRR = GPIO_BSRR_BR_5;
    }
}

void MTC_Init(void)
{
    MTC_CurrentMotorDirection = MTC_MOTOR_DIRECTION_FWD;
    MTC_SetMotorDirection(MTC_MOTOR_DIRECTION_FWD);
    MTC_ShutdownMotor();
    GPIOA->BSRR = GPIO_BSRR_BS_8; // -> reset
}

void MTC_SetMotorDirection(uint8_t direction)
{
    if(MTC_MOTOR_DIRECTION_FWD == direction)
    {
        // set motor forward:
        MTC_ShiftMotorDirToOutput(MTC_MOTOR_DIRECTION_FWD);
        MTC_CurrentMotorDirection = MTC_MOTOR_DIRECTION_FWD;
    }
    else
    {
        MTC_ShiftMotorDirToOutput(MTC_MOTOR_DIRECTION_RWD);
        MTC_CurrentMotorDirection = MTC_MOTOR_DIRECTION_RWD;
    }
}

uint8_t MTC_GetMotorDirection(void)
{
    return MTC_CurrentMotorDirection;
}


void MTC_SetMotorSpeed(uint16_t speed)
{
    if(speed <= MTC_CONTROL_REGISTER->ARR)
    {
        MTC_CONTROL_REGISTER->CCR3 = speed;
    }
}

void MTC_ShutdownMotor(void)
{
    MTC_CONTROL_REGISTER->CCR3 = 0u;
}
