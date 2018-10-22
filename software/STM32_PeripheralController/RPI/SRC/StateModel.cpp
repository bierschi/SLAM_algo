/*
 * StateModel.cpp
 *
 *  Created on: 22.10.2018
 *      Author: grauvogs
 */

#include <StateModel.h>
#include <math.h>

// currently 10cm deviation
#define MAX_ALLOWED_DEVIATION_M	0.01f

StateModel::StateModel() {
	ptrPathGroup = new PathGroup();
	posUpdater = new PositionUpdater();
}

bool StateModel::newPathAvailable(void) {
	if (NULL != ptrPathGroup) {
		return ptrPathGroup->getPathChanged();
	} else {
		return false;
	}
}

void StateModel::calcNextState(void) {

	PositionStructureType position = {0};
	position = posUpdater->getPosition();
	PathTravel *currentTarget = ptrPathGroup->getPathTravelFromIndex(currentPathTravelIndex);

	switch (currentState) {
	case STATE_IDLE:
		// wait for next path list
		if (newPathAvailable()) {
			currentPathTravelIndex = 0;
			currentState = STATE_FETCH_PATHS;
		}
		break;
	case STATE_FETCH_PATHS:
		// calculate next direction
		ptrPathGroup->determinePathTravels("./path.txt");
		break;
	case STATE_GET_NEXT_SEGMENT:

		break;
	case STATE_TRAVEL:
		// travel to next position and watch for reached destination
		// get next path to travel
		if((MAX_ALLOWED_DEVIATION_M > abs(position.x - currentTarget->getTargetX()))
				&& (MAX_ALLOWED_DEVIATION_M > abs(position.y - currentTarget->getTargetY())))
		{
			currentState = STATE_GET_NEXT_SEGMENT;
		}
		break;
	case STATE_CLEAR_STATES:
		// clear path travel states and prepare for next iteration
		break;
	}
}

