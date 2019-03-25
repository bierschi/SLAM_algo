/*
 * MPUSensor.c
 *
 *  Created on: 23.08.2018
 *      Author: stefan
 */

#include "MPUSensor.h"

#define MPU_DEVICE_ADDRESS (0x68<<1)
#define MPU_REQUEST_DATA_COMMAND 0x3B
#define MPU_DLPF_REGISTER 0x1A
#define MPU_GYRO_CONFIG_REGISTER 0x1B
#define MPU_ACCEL_CONFIG_REGISTER 0x1C
#define MPU_PWR_MNGMT_REGISTER 0x6B
#define MPU_PAYLOAD_SIZE 14u
#define MPU_AVERAGE_CYCLES 50

#define MPU_I2C_TIMEOUT	5u

/* acceleration values: */
volatile LateralAccelValueRawType CurrentRaw_X_Accel;
volatile LateralAccelValueRawType CurrentRaw_Y_Accel;
volatile LateralAccelValueRawType CurrentRaw_Z_Accel;

volatile AngularVelValueRawType CurrentRaw_XR_Vel;
volatile AngularVelValueRawType CurrentRaw_YR_Vel;
volatile AngularVelValueRawType CurrentRaw_ZR_Vel;

volatile LateralAccelValuePhysType CurrentPhys_X_Accel;
volatile LateralAccelValuePhysType CurrentPhys_Y_Accel;
volatile LateralAccelValuePhysType CurrentPhys_Z_Accel;

volatile AngularVelValuePhysType CurrentPhys_XR_Vel;
volatile AngularVelValuePhysType CurrentPhys_YR_Vel;
volatile AngularVelValuePhysType CurrentPhys_ZR_Vel;

LateralAccelValuePhysType offsetX, offsetY, offsetZ;
AngularVelValuePhysType offsetXR, offsetYR, offsetZR;

uint8_t payloadBufferI2C[15] = {0x0};

/* velocity values: */

void MPU_SetPhysLateralAccelerations(void)
{
    CurrentPhys_X_Accel = (((LateralAccelValuePhysType) CurrentRaw_X_Accel) * MPU_LATERAL_ACCEL_CONV_FACTOR) - offsetX;
    CurrentPhys_Y_Accel = (((LateralAccelValuePhysType) CurrentRaw_Y_Accel) * MPU_LATERAL_ACCEL_CONV_FACTOR) - offsetY;
    CurrentPhys_Z_Accel = (((LateralAccelValuePhysType) CurrentRaw_Z_Accel) * MPU_LATERAL_ACCEL_CONV_FACTOR) - offsetZ;
}

void MPU_SetPhysAngularVelocities(void)
{
    CurrentPhys_XR_Vel = (((AngularVelValuePhysType) CurrentRaw_XR_Vel) * MPU_ANGULAR_VEL_CONV_FACTOR) - offsetXR;
    CurrentPhys_YR_Vel = (((AngularVelValuePhysType) CurrentRaw_YR_Vel) * MPU_ANGULAR_VEL_CONV_FACTOR) - offsetYR;
    CurrentPhys_ZR_Vel = (((AngularVelValuePhysType) CurrentRaw_ZR_Vel) * MPU_ANGULAR_VEL_CONV_FACTOR) - offsetZR;
}

void MPU_GetPhysLateralAccelerations(LateralAccelValuePhysType * currentXAccel, LateralAccelValuePhysType * currentYAccel, LateralAccelValuePhysType * currentZAccel)
{
    (*currentXAccel) = CurrentPhys_X_Accel;
    (*currentYAccel) = CurrentPhys_Y_Accel;
    (*currentZAccel) = CurrentPhys_Z_Accel;
}

void MPU_GetPhysAngularVelocity(AngularVelValuePhysType * currentXRVel, AngularVelValuePhysType * currentYRVel, AngularVelValuePhysType * currentZRVel)
{
    (*currentXRVel) = CurrentPhys_XR_Vel;
    (*currentYRVel) = CurrentPhys_YR_Vel;
    (*currentZRVel) = CurrentPhys_ZR_Vel;
}

void MPU_Calibrate(void)
{
    uint16_t temp = 0u;
    int16_t *ptrToTemp = (int16_t *) &temp;

    int32_t averageXvalues = 0, averageYvalues = 0, averageZvalues = 0;
    int32_t averageXRvalues = 0, averageYRvalues = 0, averageZRvalues = 0;

    for(int cycles = 0; cycles < MPU_AVERAGE_CYCLES; cycles++)
    {
        // get values from sensor
        HAL_I2C_Mem_Read(&hi2c1, MPU_DEVICE_ADDRESS, MPU_REQUEST_DATA_COMMAND, I2C_MEMADD_SIZE_8BIT, payloadBufferI2C, 14, 50u);

        temp = ((uint16_t)payloadBufferI2C[1] | (uint16_t)(payloadBufferI2C[0]) << 8);
        averageXvalues += (int32_t) (*ptrToTemp);
        temp = ((uint16_t)payloadBufferI2C[3] | (uint16_t)(payloadBufferI2C[2]) << 8);
        averageYvalues += (int32_t) (*ptrToTemp);
        temp = ((uint16_t)payloadBufferI2C[5] | (uint16_t)(payloadBufferI2C[4]) << 8);
        averageZvalues += (int32_t) (*ptrToTemp);
        temp = ((uint16_t)payloadBufferI2C[9] | (uint16_t)(payloadBufferI2C[8]) << 8);
        averageXRvalues += (int32_t) (*ptrToTemp);
        temp = ((uint16_t)payloadBufferI2C[11] | (uint16_t)(payloadBufferI2C[10]) << 8);
        averageYRvalues += (int32_t) (*ptrToTemp);
        temp = ((uint16_t)payloadBufferI2C[13] | (uint16_t)(payloadBufferI2C[12]) << 8);
        averageZRvalues += (int32_t) (*ptrToTemp);

        HAL_Delay(5);
    }

    averageXRvalues /= MPU_AVERAGE_CYCLES;
    averageXvalues /= MPU_AVERAGE_CYCLES;
    averageYRvalues /= MPU_AVERAGE_CYCLES;
    averageYvalues /= MPU_AVERAGE_CYCLES;
    averageZRvalues /= MPU_AVERAGE_CYCLES;
    averageZvalues /= MPU_AVERAGE_CYCLES;

    offsetX = 0.0f; offsetY = 0.0f; offsetZ = 0.0f;
    offsetXR = 0.0f; offsetYR = 0.0f; offsetZR = 0.0f;

    // finally calculate offset:
    offsetX = ((LateralAccelValuePhysType) averageXvalues) * MPU_LATERAL_ACCEL_CONV_FACTOR;
    offsetY = ((LateralAccelValuePhysType) averageYvalues) * MPU_LATERAL_ACCEL_CONV_FACTOR;
    offsetZ = ((LateralAccelValuePhysType) averageZvalues) * MPU_LATERAL_ACCEL_CONV_FACTOR;
    offsetXR = ((LateralAccelValuePhysType) averageXRvalues) * MPU_ANGULAR_VEL_CONV_FACTOR;
    offsetYR = ((LateralAccelValuePhysType) averageYRvalues) * MPU_ANGULAR_VEL_CONV_FACTOR;
    offsetZR = ((LateralAccelValuePhysType) averageZRvalues) * MPU_ANGULAR_VEL_CONV_FACTOR;

}


void MPU_Init(void)
{
    uint8_t payload[2] = {0x0, 0x0}; // disable power down mode
    HAL_I2C_Mem_Write(&hi2c1, MPU_DEVICE_ADDRESS, MPU_PWR_MNGMT_REGISTER, I2C_MEMADD_SIZE_8BIT, payload, 1, MPU_I2C_TIMEOUT);
    payload[0] = 0x1; // enable digital lowpass filter (stage 1)
    HAL_I2C_Mem_Write(&hi2c1, MPU_DEVICE_ADDRESS, MPU_DLPF_REGISTER, I2C_MEMADD_SIZE_8BIT, payload, 1, MPU_I2C_TIMEOUT);
    payload[0] = 0x8; // set sensivity to level 1 +-(4g)
    HAL_I2C_Mem_Write(&hi2c1, MPU_DEVICE_ADDRESS, MPU_GYRO_CONFIG_REGISTER, I2C_MEMADD_SIZE_8BIT, payload, 1, MPU_I2C_TIMEOUT);
    payload[0] = 0x8; // set sensivity to level 1 +-(4g)
    HAL_I2C_Mem_Write(&hi2c1, MPU_DEVICE_ADDRESS, MPU_ACCEL_CONFIG_REGISTER, I2C_MEMADD_SIZE_8BIT, payload, 1, MPU_I2C_TIMEOUT);

    MPU_Calibrate();
}

void MPU_ReadValues(void)
{
    uint16_t temp;
    int16_t *ptrToTemp = (int16_t *) &temp;
    /* Trigge Measurement from MPU */
    HAL_I2C_Mem_Read(&hi2c1, MPU_DEVICE_ADDRESS, MPU_REQUEST_DATA_COMMAND, I2C_MEMADD_SIZE_8BIT, payloadBufferI2C, 14, MPU_I2C_TIMEOUT);

    /* write to local data structures */
    temp = ((uint16_t)payloadBufferI2C[1] | (uint16_t)(payloadBufferI2C[0]) << 8);
    CurrentRaw_X_Accel = (*ptrToTemp);
    temp = ((uint16_t)payloadBufferI2C[3] | (uint16_t)(payloadBufferI2C[2]) << 8);
    CurrentRaw_Y_Accel = (*ptrToTemp);
    temp = ((uint16_t)payloadBufferI2C[5] | (uint16_t)(payloadBufferI2C[4]) << 8);
    CurrentRaw_Z_Accel = (*ptrToTemp);
    temp = ((uint16_t)payloadBufferI2C[9] | (uint16_t)(payloadBufferI2C[8]) << 8);
    CurrentRaw_XR_Vel = (*ptrToTemp);
    temp = ((uint16_t)payloadBufferI2C[11] | (uint16_t)(payloadBufferI2C[10]) << 8);
    CurrentRaw_YR_Vel = (*ptrToTemp);
    temp = ((uint16_t)payloadBufferI2C[13] | (uint16_t)(payloadBufferI2C[12]) << 8);
    CurrentRaw_ZR_Vel = (*ptrToTemp);

    MPU_SetPhysLateralAccelerations();
    MPU_SetPhysAngularVelocities();
}

