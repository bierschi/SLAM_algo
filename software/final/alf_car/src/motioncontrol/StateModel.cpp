/*
 * StateModel.cpp
 *
 *  Created on: 22.10.2018
 *      Author: grauvogs
 */

#include <StateModel.h>
#include <unistd.h>
#include <math.h>

#include "ComStructure.h"
#include "SPILibrary.h"

// if defined, we should use the position update provided by file
#define USE_POSITION_FROM_FILE
//#define USE_THETA_FROM_FILE

#define MAX_STEERING_ANGLE_DEGREE 	45.0f // maximum degrees the steering wheel can handle
#define REVERSE_THRESHOLD_DEGREE 	90.0f // degrees to detect turn around situation

#define STATE_MACHINE_CYCLE_TIME	200000000L
#define REVERSE_WAIT_TIME			1


// global communication structures
ComStructureType COM_StructRX = {0};
ComStructureType COM_StructTX = {0};

// #########################################################################################
// helper functions:
// #########################################################################################
float getSteeringDirectionDegree(PositionStructureType &position, PathTravel &travel)
{
	float degree = 0.0f; // 0 degree as default
	float temp = 0.0f;
	float divider = 1.0f; // mapping from theoretical steering angle to real wheel angle

	// get the angle towards the target
	temp = atan2f((travel.getTargetY() - position.y), (travel.getTargetX() - position.x));

	// convert from radians to degree
	degree = (180.0f / 3.14159265358979323846f) * temp;

    degree -= position.theta;

	// check and apply bounds
	degree = fmaxf(degree, -MAX_STEERING_ANGLE_DEGREE);
	degree = fminf(degree, MAX_STEERING_ANGLE_DEGREE);

	degree /= divider;

	return degree;
}

float getCorrectedThetaDegree(float theta)
{
    float result = 0.0f;

    result = fmodf(theta, 360.0f);

    if(result >= 0.0f && result >180.0f)
    {
        result = -1.0f * (360.0f - theta);
    }
    else if(result < 0.0f && result < -180.0f)
    {
        result = (result + 360.0f);
    }

    return result;
}

float getHeadingAngleDiff(PositionStructureType &position, PathTravel &travel)
{
	float degree = 0.0f; // 0 degree as default
	float temp = 0.0f;

	// get the angle towards the target
	temp = atan2f((travel.getTargetY() - position.y), (travel.getTargetX() - position.x));

	// convert from radians to degree
	degree = (180.0f / 3.14159265358979323846f) * temp;

    degree -= position.theta;

	// check and apply bounds
	degree = fmaxf(degree, -180.0f);
	degree = fminf(degree, 180.0f);

	return degree;
}

// #########################################################################################
// class definition
// #########################################################################################

// PRIVATE:
bool StateModel::newPathAvailable(void) {
	if (NULL != ptrPathGroup) {
		return ptrPathGroup->getPathChanged(PATH_FILE);
	} else {
		return false;
	}
}

void StateModel::getConfig(void)
{
    FILE * fptr = NULL;
    char buffer[100] = {0};
    uint16_t motorspeed = COM_STEERING_SPEED_ZERO;
    uint8_t direction = COM_STEERING_DIRECTON_ZERO;
    float deviation = 0.0f;
    unsigned int indexIncrement = 1u;

    fptr = fopen(CONFIG_FILE, "r");

    // read configs from file:

    // read motor speed:
    if((NULL != fptr) && (NULL != fgets(buffer, 99, fptr)))
    {
        sscanf(buffer, "Speed:%hu", &motorspeed);
    }

    this->defaultMotorSpeed = motorspeed;

    if((NULL != fptr) && (NULL != fgets(buffer, 99, fptr)))
    {
        sscanf(buffer, "Direction:%hhu", &direction);
    }

    // store speed and direction of motor:
    this->defaultMotorDirection = direction;

    if((NULL != fptr) && (NULL != fgets(buffer, 99, fptr)))
    {
        sscanf(buffer, "AllowedDeviation:%f", &deviation);
    }

    this->maxAllowedDeviation = deviation;

    if((NULL != fptr) && (NULL != fgets(buffer, 99, fptr)))
    {
        sscanf(buffer, "PathIndexIncrement:%u", &indexIncrement);
    }

    this->pathIndexIncrement = indexIncrement;

    if(fptr != NULL) fclose(fptr);
}

void StateModel::writeUltrasonicDistancesToFile(void)
{
    FILE * fptr = NULL;
    char buffer[500] = {0};

    fptr = fopen(ULTRASONIC_FILE, "w");

    if(NULL != fptr)
    {
        snprintf(buffer, sizeof(buffer) - 1 , "USDistanceFrontLeft: %hu\nUSDistanceFrontRight: %hu\nUSDistanceRear: %hu",
        COM_StructRX.USDistanceFrontLeft, COM_StructRX.USDistanceFrontRight, COM_StructRX.USDistanceRear);
        fputs(buffer, fptr);

        fclose(fptr);
    }
}

void StateModel::writeControlStateToFile(void)
{
    FILE * fptr = NULL;
    char buffer[500] = {0};

    fptr = fopen(CONTROL_STATE_FILE, "w");

    if(NULL != fptr)
    {
        snprintf(buffer, sizeof(buffer) - 1 , "ControlState: %i", this->currentState);
        fputs(buffer, fptr);

        fclose(fptr);
    }
}

void StateModel::writeMotorStateToFile(void)
{
    FILE * fptr = NULL;
    char buffer[500] = {0};

    fptr = fopen(MOTOR_STATE_FILE, "w");

    if(NULL != fptr)
    {
        snprintf(buffer, sizeof(buffer) - 1 , "Motorspeed: %u\nSteeringAngle: %.2f", COM_StructTX.CurrentSteeringSpeed, COM_StructTX.CurrentSteeringAngle);
        fputs(buffer, fptr);

        fclose(fptr);
    }
}

// PUBLIC:
StateModel::StateModel() {
}

void StateModel::Init(void)
{
	ptrPathGroup = new PathGroup();
	posUpdater = new PositionUpdater();
	COM_StructTX.CurrentSteeringMode = COM_STEERING_MODE_MANUAL;
    getConfig();
}

void StateModel::calcNextState(void) {
	PathTravel *currentTarget = NULL;
	PositionStructureType position = {0};
	float degree = 0.0f;
	struct timespec ts_sleep = {0}, ts_remaining = {0};

	posUpdater->updatePosition();
	position = posUpdater->getPosition(); // get current position from module

// define if to use theta from file input
#ifndef USE_THETA_FROM_FILE
    position.theta = COM_StructRX.CurrentOrientation;
#endif
    position.theta = getCorrectedThetaDegree(position.theta);

// define if to use position from file input
#ifndef USE_POSITION_FROM_FILE
    position.x = COM_StructRX.CurrentPositionX;
    position.y = COM_StructRX.CurrentPositionY;
#endif

	// log position to output
	printf("Current Position: (position.x: %.2f, position.y: %.2f, theta: %.2f)\n", position.x, position.y, position.theta);

	switch (currentState) {
	case STATE_IDLE:
		// wait for next path list
		printf("Current State: IDLE\n");
		if (newPathAvailable()) {
			currentPathTravelIndex = 0;
			currentState = STATE_FETCH_PATHS;
		}
		break;

	case STATE_FETCH_PATHS:
		// calculate next direction
		printf("Current State: FETCH_PATHS\n");
		ptrPathGroup->determinePathTravels(PATH_FILE);

		if (ptrPathGroup->getNumAvailPathTravels() > 0)
			currentState = STATE_GET_NEXT_SEGMENT;
		else
			currentState = STATE_CLEAR_STATES;
		break;

	case STATE_GET_NEXT_SEGMENT:
		// get next segment on path
		printf("Current State: GET_NEXT_SEGMENT\n");

		if (ptrPathGroup->pathAtIndexAvailable(currentPathTravelIndex)) {
			currentTarget = ptrPathGroup->getPathTravelFromIndex(currentPathTravelIndex);

            if(REVERSE_THRESHOLD_DEGREE < fabsf(getHeadingAngleDiff(position, (*currentTarget))))
            {
                currentState = STATE_REVERSE_BACKWARD;
            }
            else
            {
                currentState = STATE_TRAVEL;
            }
		} else {
			currentState = STATE_CLEAR_STATES;
		}

		break;
	case STATE_TRAVEL:

		printf("Current State: TRAVEL\n");

		// travel to next position and watch for reached destination
		// get next path to travel
		currentTarget = ptrPathGroup->getPathTravelFromIndex(currentPathTravelIndex);

		// always track orientation towards target:
		degree = getSteeringDirectionDegree(position, (*currentTarget));
		COM_StructTX.CurrentSteeringAngle = degree;
        COM_StructTX.CurrentSteeringDirection = this->defaultMotorDirection;
        COM_StructTX.CurrentSteeringSpeed = this->defaultMotorSpeed;

		if ((this->maxAllowedDeviation > abs(position.x - currentTarget->getTargetX()))
				&& (this->maxAllowedDeviation > abs(position.y - currentTarget->getTargetY()))) {
			// stop traveling if target position is reached
			currentState = STATE_GET_NEXT_SEGMENT;
			COM_StructTX.CurrentSteeringSpeed = 0u; // shutdown motor
			currentPathTravelIndex += pathIndexIncrement; // get next path segment, if available
		}

		spiSend(COM_StructTX, COM_StructRX);
        writeUltrasonicDistancesToFile();
		break;

	case STATE_CLEAR_STATES:

		printf("Current State: CLEAR_STATES\n");
		ptrPathGroup->clearPathTravels();
		currentState = STATE_IDLE;

		// clear path travel states and prepare for next iteration
		break;

    case STATE_REVERSE_BACKWARD:

        printf("Current State: REVERSE_BACKWARD\n");
        ts_sleep.tv_sec = REVERSE_WAIT_TIME; // delay

        currentTarget = ptrPathGroup->getPathTravelFromIndex(currentPathTravelIndex);

        COM_StructTX.CurrentSteeringDirection = COM_STEERING_DIRECTION_REVERSE;
        COM_StructTX.CurrentSteeringSpeed = this->defaultMotorSpeed;
        COM_StructTX.CurrentSteeringAngle = MAX_STEERING_ANGLE_DEGREE;
        currentState = STATE_REVERSE_FORWARD;

		if(fabsf(getHeadingAngleDiff(position, (*currentTarget))) < REVERSE_THRESHOLD_DEGREE)
		{
            COM_StructTX.CurrentSteeringDirection = COM_STEERING_DIRECTION_FORWARD;
			COM_StructTX.CurrentSteeringSpeed = COM_STEERING_SPEED_ZERO;
			COM_StructTX.CurrentSteeringAngle = 0.0f;
			currentState = STATE_TRAVEL;
		}

		spiSend(COM_StructTX, COM_StructRX);
		nanosleep(&ts_sleep, &ts_remaining);
        break;
    case STATE_REVERSE_FORWARD:

        printf("Current State: REVERSE_FORWARD\n");
        ts_sleep.tv_sec = REVERSE_WAIT_TIME; // delay

        currentTarget = ptrPathGroup->getPathTravelFromIndex(currentPathTravelIndex);

        COM_StructTX.CurrentSteeringDirection = COM_STEERING_DIRECTION_FORWARD;
        COM_StructTX.CurrentSteeringSpeed = this->defaultMotorSpeed;
        COM_StructTX.CurrentSteeringAngle = -MAX_STEERING_ANGLE_DEGREE;
        currentState = STATE_REVERSE_BACKWARD;

        if(fabsf(getHeadingAngleDiff(position, (*currentTarget))) < REVERSE_THRESHOLD_DEGREE)
        {
            COM_StructTX.CurrentSteeringDirection = COM_STEERING_DIRECTION_FORWARD;
            COM_StructTX.CurrentSteeringSpeed = COM_STEERING_SPEED_ZERO;
            COM_StructTX.CurrentSteeringAngle = 0.0f;
            currentState = STATE_TRAVEL;
        }

        spiSend(COM_StructTX, COM_StructRX);
        nanosleep(&ts_sleep, &ts_remaining);
        break;

	default:
		currentState = STATE_CLEAR_STATES;
		break;
	}

    this->writeControlStateToFile();
    this->writeMotorStateToFile();

}

void StateModel::Main(void)
{
	struct timespec ts_sleep = {0}, ts_remaining = {0};
	ts_sleep.tv_nsec = STATE_MACHINE_CYCLE_TIME; // 200 ms delay
	Init();

	while(1)
	{
		printf("Calculating Next State...\n");
		calcNextState();
		nanosleep(&ts_sleep, &ts_remaining);
	}
}

