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
#define USE_THETA_FROM_FILE

#define MAX_STEERING_ANGLE_DEGREE 	40.0f // maximum degrees the steering wheel can handle
#define REVERSE_THRESHOLD_DEGREE 	40.0f // degrees to detect turn around situation

#define STATE_MACHINE_CYCLE_TIME	200000000L	// delay between state changes in nanoseconds
#define REVERSE_WAIT_TIME			1			// wait time for reversing in seconds

#define SWITCH_MOTOR_OFF()          do { \
    COM_StructTX.CurrentSteeringAngle = COM_STEERING_ANGLE_ZERO; \
    COM_StructTX.CurrentSteeringDirection = this->defaultMotorDirection; \
    COM_StructTX.CurrentSteeringSpeed = COM_STEERING_SPEED_ZERO; \
    } while(0);

// global communication structures
ComStructureType COM_StructRX = {0};
ComStructureType COM_StructTX = {0};

// #########################################################################################
// helper functions:
// #########################################################################################
float getCorrectedThetaDegreeAccel(float theta)
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

float getCorrectedThetaDegreeLidar(float theta)
{
    float result = 0.0f;

    if(theta < 180.0f)
    {
        result = theta * -1.0f;
    }
    else
    {
        result = 360.0f - theta;
    }

    return result;
}

float getHeadingAngleDiff(PositionStructureType &position, PathTravel &travel, bool useSteering)
{
	float degree = 0.0f; // 0 degree as default
	float absTargetAngle = 0.0f;
	float relTargetAngle = 0.0f;
	float maxAngle = 180.0f;

	// get the angle towards the target
	absTargetAngle = atan2f((travel.getTargetY() - position.y), (travel.getTargetX() - position.x));

	absTargetAngle = (180.0f / 3.14159265358979323846f) * absTargetAngle;
	
	printf("Zielwinkel: %.2f", absTargetAngle);

	if((360.0f - absTargetAngle + position.theta) > 180.0f)
	{
		if((0.0f - absTargetAngle + position.theta) > 180.0f)
		{
			relTargetAngle = 0.0f - 360.0f - absTargetAngle + position.theta;
		}
		else
		{
			relTargetAngle = 0.0f - absTargetAngle + position.theta;
		}
	}
	else
	{
		relTargetAngle = 360.0f - absTargetAngle + position.theta;
	}

	if(useSteering) maxAngle = MAX_STEERING_ANGLE_DEGREE;
	// check and apply bounds
	degree = fmaxf(relTargetAngle, -maxAngle);
	degree = fminf(relTargetAngle, maxAngle);

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
    COM_StructTX.CurrentSteeringAngle = COM_STEERING_ANGLE_ZERO;
    COM_StructTX.CurrentSteeringSpeed = COM_STEERING_SPEED_ZERO;
    COM_StructTX.CurrentSteeringDirection = COM_STEERING_DIRECTION_FORWARD;
    getConfig();

    // check if 360 degree area scan at program startup should be made
    if(this->scanAtStart)
    {
        this->currentState = STATE_SCAN_AREA;
        this->lastPosition.theta = 20.0f;
    }
}

void StateModel::calcNextState(void) {
	PathTravel *currentTarget = NULL;
	PositionStructureType position = {0};
	float degree = 0.0f;
	struct timespec ts_sleep = {0}, ts_remaining = {0};

#ifndef SPIINTERFACE_STANDALONE
	position = posUpdater->getPosition(); // get current position from module / file
#else
	//posUpdater->updatePosition();
    //position = 
#endif

    spiSend(COM_StructTX, COM_StructRX);

// define if to use theta from file input
#ifndef USE_THETA_FROM_FILE
    position.theta = COM_StructRX.CurrentOrientation;
    position.theta = getCorrectedThetaDegreeAccel(position.theta);
#else
    position.theta = getCorrectedThetaDegreeLidar(position.theta);
#endif

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
		printf("Press any key to continue...\n");
		getchar();
		if (newPathAvailable()) {
			currentPathTravelIndex = 1;
			currentState = STATE_FETCH_PATHS;
		}

        SWITCH_MOTOR_OFF();
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

			// if the absolute angle difference to the target is greater than threshold,
			// we have to turn around first -> change to state reverse
            if(REVERSE_THRESHOLD_DEGREE < fabsf(getHeadingAngleDiff(position, (*currentTarget), false)))
            {
                currentState = STATE_REVERSE_BACKWARD;

                // turn to the target, select direction of movement:
				if(0.0f < getHeadingAngleDiff(position, (*currentTarget), false))
				{
					COM_StructTX.CurrentSteeringAngle = MAX_STEERING_ANGLE_DEGREE;
				}
				else
				{
					COM_StructTX.CurrentSteeringAngle = -MAX_STEERING_ANGLE_DEGREE;
				}
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
		degree = getHeadingAngleDiff(position, (*currentTarget), true);
		COM_StructTX.CurrentSteeringAngle = degree;
        COM_StructTX.CurrentSteeringDirection = this->defaultMotorDirection;
        COM_StructTX.CurrentSteeringSpeed = this->defaultMotorSpeed;

		if ((this->maxAllowedDeviation > abs(position.x - currentTarget->getTargetX()))
				&& (this->maxAllowedDeviation > abs(position.y - currentTarget->getTargetY()))) {
			// stop traveling if target position is reached
			currentState = STATE_GET_NEXT_SEGMENT;
			currentPathTravelIndex += pathIndexIncrement; // get next path segment, if available

            SWITCH_MOTOR_OFF();
		}

		break;

	case STATE_CLEAR_STATES:

		printf("Current State: CLEAR_STATES\n");
		ptrPathGroup->clearPathTravels();
		currentState = STATE_IDLE;

		// clear path travel states and prepare for next iteration
		break;

    case STATE_REVERSE_BACKWARD:

    	printf("Current State: REVERSE_BACkWARD\n");
		ts_sleep.tv_sec = REVERSE_WAIT_TIME; // delay

		currentTarget = ptrPathGroup->getPathTravelFromIndex(currentPathTravelIndex);

		COM_StructTX.CurrentSteeringDirection = COM_STEERING_DIRECTION_REVERSE;
		COM_StructTX.CurrentSteeringSpeed = this->defaultMotorSpeed;

		// decide steering angle, positive or negative (positive means right turn, negative left turn)
		COM_StructTX.CurrentSteeringAngle *= -1.0f;

		if(fabsf(getHeadingAngleDiff(position, (*currentTarget), false)) < REVERSE_THRESHOLD_DEGREE)
		{
			SWITCH_MOTOR_OFF();
			currentState = STATE_TRAVEL;
		}
		else
		{
			currentState = STATE_REVERSE_FORWARD;
			nanosleep(&ts_sleep, &ts_remaining);
		}

		break;

    case STATE_REVERSE_FORWARD:

        printf("Current State: REVERSE_FORWARD\n");
        ts_sleep.tv_sec = REVERSE_WAIT_TIME; // delay

        currentTarget = ptrPathGroup->getPathTravelFromIndex(currentPathTravelIndex);

        COM_StructTX.CurrentSteeringDirection = COM_STEERING_DIRECTION_FORWARD;
        COM_StructTX.CurrentSteeringSpeed = this->defaultMotorSpeed;

        // decide steering angle, positive or negative (positive means left turn, negative right turn)
        COM_StructTX.CurrentSteeringAngle *= -1.0f;

		if(fabsf(getHeadingAngleDiff(position, (*currentTarget), false)) < REVERSE_THRESHOLD_DEGREE)
		{
			SWITCH_MOTOR_OFF();
			currentState = STATE_TRAVEL;
		}
		else
		{
			currentState = STATE_REVERSE_BACKWARD;
			nanosleep(&ts_sleep, &ts_remaining);
		}

        break;

    case STATE_SCAN_AREA:

    	printf("Current State: SCAN_AREA\n");
    	ts_sleep.tv_sec = REVERSE_WAIT_TIME; // delay

    	this->turnAroundTheta += fabsf(fabsf(position.theta) - fabsf(this->lastPosition.theta));

    	if(COM_StructTX.CurrentSteeringDirection == COM_STEERING_DIRECTION_FORWARD)
    	{
    		COM_StructTX.CurrentSteeringDirection = COM_STEERING_DIRECTION_REVERSE;
    		COM_StructTX.CurrentSteeringAngle = MAX_STEERING_ANGLE_DEGREE;
    	}
    	else
    	{
    		COM_StructTX.CurrentSteeringDirection = COM_STEERING_DIRECTION_FORWARD;
    		COM_StructTX.CurrentSteeringAngle = -MAX_STEERING_ANGLE_DEGREE;
    	}

    	COM_StructTX.CurrentSteeringSpeed = this->defaultMotorSpeed;

    	if(this->turnAroundTheta > 360.0f)
    	{
    		SWITCH_MOTOR_OFF();
    		currentState = STATE_CLEAR_STATES;
    		this->turnAroundTheta = 0.0f;
    	}
    	else
    	{
    		currentState = STATE_SCAN_AREA;
    		nanosleep(&ts_sleep, &ts_remaining);
    	}

    	break;

	default:
		currentState = STATE_CLEAR_STATES;
		break;
	}

	// store last position (theta)
	this->lastPosition = position;

    // send motor commands to board
    //spiSend(COM_StructTX, COM_StructRX);

    this->writeControlStateToFile();
    this->writeMotorStateToFile();
    this->writeUltrasonicDistancesToFile();

}

void StateModel::Main(void)
{
	struct timespec ts_sleep = {0}, ts_remaining = {0};
	ts_sleep.tv_nsec = STATE_MACHINE_CYCLE_TIME; // 200 ms delay

    calcNextState();

	nanosleep(&ts_sleep, &ts_remaining);
}

void StateModel::Close(void)
{
    SWITCH_MOTOR_OFF();
	spiSend(COM_StructTX, COM_StructRX);
    printf("MotorShutdown!\n");
}

void StateModel::setScanAtStartup(bool scanAtStart)
{
	this->scanAtStart = scanAtStart;
	this->currentState = STATE_SCAN_AREA;
}

void StateModel::updatePosition(float xpos, float ypos, float theta)
{
    if(NULL != this->posUpdater)
    {
        this->posUpdater.updatePosition(xpos, ypos, theta);
    }
}

void StateModel::updatePathTravels(std::string &pathTravels)
{
    if(NULL != this->ptrPathGroup)
    {
        this->ptrPathGroup->determinePathTravels(pathTravels);
    }
}

bool StateModel::isBusy(void)
{
    if(currentState != STATE_IDLE) {
        return true; // controller is busy, perhaps we are currently travelling along a path
    } else {
        return false;
    }
}
