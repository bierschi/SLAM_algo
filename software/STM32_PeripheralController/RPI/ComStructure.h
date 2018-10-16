/*
 * ComStructure.h
 *
 * Defines the global communication struct transmitted over SPI (DMA)
 *
 * The Raspberry Pi also needs the copy of this header, thus the sent data has to be in sync
 * perfectly.
 *
 *  Created on: 09.09.2018
 *      Author: stefan
 */

#ifndef COMSTRUCTURE_H
#define COMSTRUCTURE_H

// define some available steering modes on STM32 board
#define COM_STEERING_MODE_AUTO 1u
#define COM_STEERING_MODE_MANUAL 2u

// define some initialization values for the communication structure
#define COM_STEERING_SPEED_ZERO 0u
#define COM_STEERING_DIRECTON_ZERO 0u
#define COM_STEERING_ANGLE_ZERO 0.0f

// adjust packing size for SPI Transfer Data Compression purposes
#pragma pack(push, 1)

typedef struct ComStructure{
	// select steering mode AUTO or MANUAL
	uint8_t CurrentSteeringMode;

	// Desired Steering Speed in 0...1000
	uint16_t CurrentSteeringSpeed;

	// motor forward or backward (0 ... 1)
	uint8_t CurrentSteeringDirection;

	// Desired Steering Angle in Degree (rel. for manual mode, unit: degree)
	float CurrentSteeringAngle;

	// target point x (rel. for automatic steering, unit: mm)
	float Target_X;
	// target point y (rel. for automatic steering, unit:mm)
	float Target_Y;

	// current orientation angle in degree
	float CurrentOrientation;
	// current robot position x in meters:
	float CurrentPositionX;
	// current robot position y in meters
	float CurrentPositionY;

	// ultrasonic distance sensor values in front left (unit: mm)
	uint16_t USDistanceFrontLeft;
	// ultrasonic distance sensor values in front right (unit: mm)
	uint16_t USDistanceFrontRight;
	// ultrasonic distance sensor values in rear (unit: mm)
	uint16_t USDistanceRear;
} ComStructureType;

#pragma pack(pop)



#endif /* SRC_COMMODULE_COMSTRUCTURE_H_ */
