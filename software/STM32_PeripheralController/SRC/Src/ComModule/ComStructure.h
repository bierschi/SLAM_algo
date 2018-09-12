/*
 * ComStructure.h
 *
 *  Created on: 09.09.2018
 *      Author: stefan
 */

#ifndef COMSTRUCTURE_H
#define COMSTRUCTURE_H


#define COM_STEERING_MODE_AUTO 1u
#define COM_STEERING_MODE_MANUAL 2u

#define COM_STEERING_SPEED_ZERO 0u
#define COM_STEERING_DIRECTON_ZERO 0u
#define COM_STEERING_ANGLE_ZERO 0.0f

// adjust packing size for SPI Transfer Data Compression purposes
#pragma pack(push, 1)

typedef struct {
	uint8_t CurrentSteeringMode; // AUTO, MANUAL
	uint16_t CurrentSteeringSpeed; // Desired Steering Speed in 0...1000
	uint8_t CurrentSteeringDirection; // forward or backward
	float CurrentSteeringAngle; // Desired Steering Angle in Degree

	float Target_X;
	float Target_Y;
} ComStructureType;

#pragma pack(pop)



#endif /* SRC_COMMODULE_COMSTRUCTURE_H_ */
