/*
 * MouseSensor.c
 *
 *  Created on: 03.06.2016
 *      Author: stefan
 */

#include "MouseSensor.h"
#include "stm32f334x8.h"
#include "PeripheralHandles.h"

#define DELAY_120_US    620u
#define DELAY_3_US      6u

/* ADNS2051 register definitions */
#define REG_PRODUCT_ID 0x00
#define REG_REVISION_ID 0x01
#define REG_MOTION 0x02
#define REG_DELTA_X 0x03
#define REG_DELTA_Y 0x04
#define REG_SQUAL 0x05
#define REG_AVERAGE_PIXEL 0x06
#define REG_MAXIMUM_PIXEL 0x07
#define REG_CONFIG_BITS 0x0A
#define REG_DATA_OUT_LOWER 0x0C
#define REG_DATA_OUT_UPPER 0x0D
#define REG_SHUTTER_LOWER 0x0E
#define REG_SHUTTER_UPPER 0x0F
#define REG_FRAME_PERIOD_LOWER 0x10
#define REG_FRAME_PERIOD_UPPER 0x11

/* currently acquired x and y deviations from mouse sensor */
int32_t MSC_CurrentYDelta = 0u;
int32_t MSC_CurrentXDelta = 0u;

/* global x and y positions */
int32_t MSC_CurrentPositionX = 0;
int32_t MSC_CurrentPositionY = 0;

float MSC_CurrentVelocityX = 0.0f;
float MSC_CurrentVelocityY = 0.0f;

float MSC_CurrentOrientation = 0.0f;

void MSC_delayMicroseconds(uint16_t microseconds)
{
    uint16_t i = 0u;

    for (; i < microseconds; i++)
        ;
}

uint8_t MSC_SensorReadRegister(uint8_t address)
{
    uint8_t i = 128;
    uint8_t res = 0;

    /* set SDIO pin to output mode: */
    GPIOA->MODER |= GPIO_MODER_MODER8_0;

    for (; i > 0; i >>= 1)
    {
        GPIOB->BSRR = GPIO_BSRR_BR_10;
        if ((address & i) != 0u)
        {
            GPIOA->BSRR = GPIO_BSRR_BS_8;
        }
        else
        {
            GPIOA->BSRR = GPIO_BSRR_BR_8;
        }
        GPIOB->BSRR = GPIO_BSRR_BS_10;
    }

    /* set SDIO pin to input mode: */
    GPIOA->MODER &= ~GPIO_MODER_MODER8_0;

    MSC_delayMicroseconds(DELAY_120_US);  // 1240 = (100e-6/(62e6 ^-1))/5 instructions

    for (uint8_t i = 128; i > 0u; i >>= 1)
    {
        GPIOB->BSRR = GPIO_BSRR_BR_10;
        GPIOB->BSRR = GPIO_BSRR_BR_10;
        if ((GPIOA->IDR & GPIO_IDR_8) != 0u)
        {
            res |= i;
        }
        else
        {
            res &= ~i;
        }
        GPIOB->BSRR = GPIO_BSRR_BS_10;
    }

    return res;
}

void MSC_SensorWriteRegister(uint8_t address, uint8_t data)
{
    address |= 0x80;  // MSB indicates write mode.
    GPIOA->MODER |= GPIO_MODER_MODER8_0;

    for (uint8_t i = 128u; i > 0u; i >>= 1)
    {
        GPIOB->BSRR = GPIO_BSRR_BR_10;
        if ((address & i) != 0)
        {
            GPIOA->BSRR = GPIO_BSRR_BS_8;
        }
        else
        {
            GPIOA->BSRR = GPIO_BSRR_BR_8;
        }
        GPIOB->BSRR = GPIO_BSRR_BS_10;
    }

    for (uint8_t i = 128u; i > 0u; i >>= 1)
    {
        GPIOB->BSRR = GPIO_BSRR_BR_10;
        if ((data & i) != 0u)
        {
            GPIOA->BSRR = GPIO_BSRR_BS_8;
        }
        else
        {
            GPIOA->BSRR = GPIO_BSRR_BR_8;
        }
        GPIOB->BSRR = GPIO_BSRR_BS_10;
    }

    MSC_delayMicroseconds(DELAY_120_US);  // tSWW, tSWR = 100us min.
}

void MSC_SensorReset()
{
    /* currently acquired x and y deviations from mouse sensor */
    MSC_CurrentYDelta = 0u;
    MSC_CurrentXDelta = 0u;

    /* global x and y positions */
    MSC_CurrentPositionX = 0;
    MSC_CurrentPositionY = 0;

    MSC_CurrentVelocityX = 0.0f;
    MSC_CurrentVelocityY = 0.0f;

    MSC_CurrentOrientation = 0.0f;

    GPIOB->MODER |= GPIO_MODER_MODER10_0;
    GPIOA->MODER |= GPIO_MODER_MODER8_0;
    GPIOB->BSRR |= GPIO_BSRR_BS_10;
    MSC_delayMicroseconds(2u * DELAY_3_US);

    uint8_t configure = MSC_SensorReadRegister(REG_CONFIG_BITS);
    //configure &= 0xEE; // Don't sleep (LED always powered on).
    //configure |= B00010000; // Don't sleep (LED always powered on).
    //MSC_SensorWriteRegister(REG_CONFIG_BITS, configure);

    MSC_delayMicroseconds(DELAY_120_US);

    /* freeze register values for Y and X motion */
    MSC_SensorReadRegister(REG_MOTION);

    MSC_delayMicroseconds(DELAY_3_US);

    MSC_SensorReadRegister(REG_DELTA_Y);

    MSC_delayMicroseconds(DELAY_3_US);

    MSC_SensorReadRegister(REG_DELTA_X);
}

void MSC_SensorAcquirePosition(void)
{
    uint8_t y_delta;
    uint8_t x_delta;
    /* uint8_t motion; */

    /* freeze register values for Y and X motion */
    MSC_SensorReadRegister(REG_MOTION);

    MSC_delayMicroseconds(DELAY_3_US);

    y_delta = MSC_SensorReadRegister(REG_DELTA_Y);

    MSC_delayMicroseconds(DELAY_3_US);

    x_delta = MSC_SensorReadRegister(REG_DELTA_X);

    MSC_CurrentYDelta = (int8_t) y_delta;
    MSC_CurrentXDelta = (int8_t) x_delta;

    MSC_CurrentPositionX += (int8_t) x_delta;
    MSC_CurrentPositionY += (int8_t) y_delta;

    MSC_CurrentVelocityX = (((float) (MSC_CurrentXDelta)) * (MSC_FACTOR_CPI_TO_CM)) / MSC_USED_SAMPLE_TIME;
    MSC_CurrentVelocityY = (((float) (MSC_CurrentYDelta)) * (MSC_FACTOR_CPI_TO_CM)) / MSC_USED_SAMPLE_TIME;

    /* calculate angle movement */
    MSC_CurrentOrientation = (MSC_FACTOR_CPI_TO_CM * (float) MSC_CurrentYDelta * MSC_DISTANCE_FROM_TURNPOINT)
            * (180.0f / 3.14f);
}

int32_t MSC_GetCurrentPositionX(void)
{
    return MSC_CurrentPositionX;
}

int32_t MSC_GetCurrentPositionY(void)
{
    return MSC_CurrentPositionY;
}

float MSC_GetCurrentVelocityX(void)
{
    return MSC_CurrentVelocityX;
}

float MSC_GetCurrentVelocityY(void)
{
    return MSC_CurrentVelocityY;
}
