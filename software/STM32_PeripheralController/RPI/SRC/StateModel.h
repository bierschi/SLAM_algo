/*
 * StateModel.h
 *
 *  Created on: 22.10.2018
 *      Author: grauvogs
 */

#ifndef STATEMODEL_H
#define STATEMODEL_H

#include "Path.h"

typedef enum StateEnum
{
	STATE_IDLE,
	STATE_FETCH_PATHS,
	STATE_TRAVEL,
	STATE_GET_NEXT_SEGMENT,
	STATE_CLEAR_STATES
} StateEnumType;

class StateModel
{
private:
	StateEnumType currentState = STATE_IDLE;
	// current index of traveled path group
	unsigned int currentPathTravelIndex = 0u;
	PathGroup *ptrPathGroup = NULL;

	bool newPathAvailable(void);
public:
	StateModel();

	void calcNextState(void);
};

#endif /* STATEMODEL_H */
