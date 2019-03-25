#include "SRFSensor.h"
#include "PeripheralHandles.h"
#include "Timing.h"

#define SRF_WAIT_CYCLES_FOR_SAMPLE 		(70u/(MAIN_SAMPLE_TIME_MS)) // 70 ms / application cycle (ms)

#define SRF_BROADCAST_ADDRESS			0x00 // address to reach all SRF modules
#define SRF_FRONT_LEFT_DEV_ADRESS 		(0xE0) // device adress for front left sonar
#define SRF_FRONT_RIGHT_DEV_ADRESS		(0xE1) // device adress for front right sonar
#define SRF_REAR_DEV_ADRESS				(0xE2) // device adress for rear

#define SRF_COMMAND_REGISTER			0x00 // command register
#define SRF_COMMAND_START_RANGING_CM	0x51 // starts ranging in centimeters, will take 65 ms to complete
#define SRF_REGISTER_1ST_ECHO_HIGH		0x02 // high byte of 1st echo
#define SRF_REGISTER_1ST_ECHO_LOW		0x03 // low byte of 1st echo

SRF_DistanceType FrontLeftDistance;
SRF_DistanceType FrontRightDistance;
SRF_DistanceType RearDistance;

uint16_t cycleCounter;
uint8_t measurementRunning;

SRF_DistanceType SRF_GetDistanceFrontLeft(void)
{
	return FrontLeftDistance;
}

SRF_DistanceType SRF_GetDistanceFrontRight(void)
{
	return FrontRightDistance;
}

SRF_DistanceType SRF_GetDistanceRear(void)
{
	return RearDistance;
}

void SRF_Init(void)
{
    FrontLeftDistance = 0u;
    FrontRightDistance = 0u;
    RearDistance = 0u;
    cycleCounter = 0u;
    measurementRunning = 0u;
}

void SRF_MainFunction(void)
{
	uint8_t data[6] = {0x00};

    if(measurementRunning == 1u)
    {
        if(cycleCounter >= SRF_WAIT_CYCLES_FOR_SAMPLE)
        {
        	HAL_I2C_Mem_Read(&hi2c1, SRF_FRONT_LEFT_DEV_ADRESS, SRF_REGISTER_1ST_ECHO_HIGH, I2C_MEMADD_SIZE_8BIT, data, 2u, 10u);
        	FrontLeftDistance = ((((uint16_t) (data[0])) << 8) | ((uint16_t) (data[1])));
        	HAL_I2C_Mem_Read(&hi2c1, SRF_FRONT_RIGHT_DEV_ADRESS, SRF_REGISTER_1ST_ECHO_HIGH, I2C_MEMADD_SIZE_8BIT, data, 2u, 10u);
        	FrontRightDistance = ((((uint16_t) (data[0])) << 8) | ((uint16_t) (data[1])));
        	HAL_I2C_Mem_Read(&hi2c1, SRF_REAR_DEV_ADRESS, SRF_REGISTER_1ST_ECHO_HIGH, I2C_MEMADD_SIZE_8BIT, data, 2u, 10u);
        	RearDistance = ((((uint16_t) (data[0])) << 8) | ((uint16_t) (data[1])));

        	cycleCounter = 0u;
        	measurementRunning = 0u;
        }
        else
        {
        	cycleCounter++;
        }

    }
    else
    {
    	// start new SRF08 measurement
    	data[0] = SRF_COMMAND_START_RANGING_CM;
    	HAL_I2C_Mem_Write(&hi2c1, SRF_BROADCAST_ADDRESS, SRF_COMMAND_REGISTER, I2C_MEMADD_SIZE_8BIT, data, 1u, 10u);
    	measurementRunning = 1u;
    	cycleCounter = 0u;
    }
}
