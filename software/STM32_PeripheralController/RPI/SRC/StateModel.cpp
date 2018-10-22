/*
 * StateModel.cpp
 *
 *  Created on: 22.10.2018
 *      Author: grauvogs
 */

#include <StateModel.h>

StateModel::StateModel() {
	// TODO Auto-generated constructor stub

}

void StateModel::calcNextState(void)
	{
		switch(currentState)
		{
		case STATE_IDLE:
			// wait for next path list
			break;
		case STATE_CALCULATE:
			// calculate next direction
			break;
		case STATE_TRAVELLING:
			// travel to next position and watch for reached destination
			break;
		}
	}

