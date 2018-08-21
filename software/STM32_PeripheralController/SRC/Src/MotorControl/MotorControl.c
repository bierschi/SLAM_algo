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

void MTC_Init(void)
{
    MTC_CurrentMotorDirection = MTC_MOTOR_DIRECTION_FWD;

    HAL_GPIO_WritePin(MOTA_1_GPIO_Port, MOTA_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTA_2_GPIO_Port, MOTA_2_Pin, GPIO_PIN_SET);
}

void MTC_SetMotorDirection(uint8_t direction)
{
    if(MTC_MOTOR_DIRECTION_FWD == direction)
    {
        HAL_GPIO_WritePin(MOTA_1_GPIO_Port, MOTA_1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTA_2_GPIO_Port, MOTA_2_Pin, GPIO_PIN_SET);
        MTC_CurrentMotorDirection = MTC_MOTOR_DIRECTION_FWD;
    }
    else
    {
        HAL_GPIO_WritePin(MOTA_1_GPIO_Port, MOTA_1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTA_2_GPIO_Port, MOTA_2_Pin, GPIO_PIN_RESET);
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
        MTC_CONTROL_REGISTER->CCR1 = speed;
    }
}

void MTC_ShutdownMotor(void)
{
    MTC_CONTROL_REGISTER->CCR1 = 0u;
    HAL_GPIO_WritePin(MOTA_1_GPIO_Port, MOTA_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTA_2_GPIO_Port, MOTA_2_Pin, GPIO_PIN_RESET);
}
