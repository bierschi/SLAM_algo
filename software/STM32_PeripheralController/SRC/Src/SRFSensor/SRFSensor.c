#include "SRFSensor.h"
#include "stm32f3xx_hal_i2c.h"
#include "PeripheralHandles.h"

#define SRF_WAIT_CYCLES_FOR_SAMPLE (70/10) // 70 ms / 10 ms

float FrontLeftDistance;
float FrontRightDistance;
float RearDistance;

uint16_t cycleCounter;
uint8_t measurementRunning;

void SRF_ReadValues(void)
{
    uint8_t data = 0x51;
    HAL_I2C_Mem_Write(&hi2c1, 0x00, 0x00, 1, &data, 1, 5); // start measurement
    measurementRunning = 1u;
    cycleCounter = 0u;
}

void SRF_GetValuesFromSensor(void)
{
    uint8_t data[2] = {0x00, 0x00};

    HAL_I2C_Mem_Read(&hi2c1, 0x00, 0x00, 1, &data, 1, 5);

}

void SRF_Init(void)
{
    FrontLeftDistance = 0.0f;
    FrontRightDistance = 0.0f;
    RearDistance = 0.0f;
    cycleCounter = 0u;
    measurementRunning = 0u;
}

void SRF_MainFunction(void)
{
    if(measurementRunning == 1u)
    {
        if(cycleCounter >= SRF_WAIT_CYCLES_FOR_SAMPLE)
        {

        }
    }
}