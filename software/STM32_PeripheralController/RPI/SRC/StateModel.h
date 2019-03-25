/*
 * StateModel.h
 *
 *  Created on: 22.10.2018
 *      Author: grauvogs
 */

#ifndef STATEMODEL_H
#define STATEMODEL_H

#include <iostream>
#include "PositionUpdater.h"
#include "ComStructure.h"
#include "Path.h"

#define PATH_FILE "../PathFinder_data/driveway.txt"
#define POSITION_FILE "../SLAM_data/position.txt"
#define MOTOR_STATE_FILE "../STM_data/motor.txt"
#define ULTRASONIC_FILE "../STM_data/ultrasonic.txt"
#define CONFIG_FILE "../STM_data/config.cfg"
#define CONTROL_STATE_FILE "../STM_data/controlStates.txt"

typedef enum StateEnum
{
	STATE_IDLE,
	STATE_FETCH_PATHS,
	STATE_TRAVEL,
	STATE_GET_NEXT_SEGMENT,
	STATE_CLEAR_STATES,
    STATE_REVERSE_BACKWARD,
    STATE_REVERSE_FORWARD,
	STATE_SCAN_AREA
} StateEnumType;

class StateModel
{
private:
	StateEnumType currentState = STATE_IDLE;
	// current index of traveled path group
	unsigned int currentPathTravelIndex = 0u;
    unsigned int pathIndexIncrement = 1u;
    bool isFirstStartup = true;
    bool scanAtStart = false;
    uint16_t defaultMotorSpeed = COM_STEERING_SPEED_ZERO;
    uint8_t defaultMotorDirection = COM_STEERING_DIRECTON_ZERO;
    float maxAllowedDeviation = 1.5f;
	PathGroup *ptrPathGroup = NULL;
	PositionUpdater *posUpdater = NULL;

	// special variables for STATE_SCAN_AREA:
	PositionStructureType lastPosition = {0};
	float turnAroundTheta = 0.0f;

	bool newPathAvailable(void);
	void getConfig(void);
    void writeUltrasonicDistancesToFile(void);
    void writeControlStateToFile(void);
    void writeMotorStateToFile(void);
public:
	StateModel();
	void calcNextState(void);
	void Init(void);
	void Main(void);
    void Close(void);
	void setScanAtStartup(bool scanAtStart);
    void updatePosition(float xpos, float ypos, float theta);
    void updatePathTravels(std::string &test);
	bool isBusy(void);
};

#endif /* STATEMODEL_H */
