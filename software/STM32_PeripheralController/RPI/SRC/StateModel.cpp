/*
 * StateModel.cpp
 *
 *  Created on: 22.10.2018
 *      Author: grauvogs
 */

#include <StateModel.h>



StateModel::StateModel() {
	ptrPathGroup = new PathGroup();
}

bool StateModel::newPathAvailable(void)
{
	return ptrPathGroup->getPathChanged();
}

void StateModel::calcNextState(void)
	{
		switch(currentState)
		{
		case STATE_IDLE:
			// wait for next path list
			if(newPathAvailable())
			{
				currentPathTravelIndex = 0;
				currentState = STATE_FETCH_PATHS;
			}
			break;
		case STATE_FETCH_PATHS:
			// calculate next direction
			ptrPathGroup->determinePathTravels("./path.txt");
			break;
		case STATE_TRAVEL:
			// travel to next position and watch for reached destination

			break;
		case STATE_GET_NEXT_SEGMENT:
			// get next path to travel
			break;
		case STATE_CLEAR_STATES:
			// clear path travel states and prepare for next iteration
			break;
		}
	}

