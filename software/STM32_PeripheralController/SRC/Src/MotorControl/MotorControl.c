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

void waitNCycles(uint16_t cycles)
{
    for(int wait = 0; wait < cycles; wait++)
    {
        __NOP;
    }
}

void MTC_ShiftMotorDirToOutput(uint8_t direction)
{
    uint8_t data = 0u;

    if(direction == MTC_MOTOR_DIRECTION_FWD)
    {
        data = 0x1;
    }
    else if(MTC_MOTOR_DIRECTION_RWD)
    {
        data = 0x4;
    }

    for(int ctr = 0; ctr < 8; ctr++)
    {
        if((data & 0x1) > 0)
        {
            GPIOA->BSRR = GPIO_BSRR_BS_9;
        }
        else
        {
            GPIOA->BSRR = GPIO_BSRR_BR_9;
        }

        data >>= 1;

        waitNCycles(5);

        // shift data out
        GPIOB->BSRR = GPIO_BSRR_BS_5;
        
        waitNCycles(10);

        GPIOB->BSRR = GPIO_BSRR_BR_5;

        waitNCycles(5);
    }

    GPIOA->BSRR = GPIO_BSRR_BR_6;

    waitNCycles(5);

    GPIOA->BSRR = GPIO_BSRR_BS_6;

    waitNCycles(5);

    GPIOA->BSRR = GPIO_BSRR_BR_6;
}

void MTC_Init(void)
{
    MTC_CurrentMotorDirection = MTC_MOTOR_DIRECTION_FWD;
    MTC_SetMotorDirection(MTC_MOTOR_DIRECTION_FWD);
    GPIOA->BSRR = GPIO_BSRR_BR_8;
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
