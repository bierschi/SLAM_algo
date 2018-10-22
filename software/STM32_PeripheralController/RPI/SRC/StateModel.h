/*
 * StateModel.h
 *
 *  Created on: 22.10.2018
 *      Author: grauvogs
 */

#ifndef STATEMODEL_H
#define STATEMODEL_H

typedef enum StateEnum
{
	STATE_IDLE,
	STATE_TRAVELLING,
	STATE_CALCULATE
} StateEnumType;

class StateModel
{
private:
	StateEnumType currentState = STATE_IDLE;
	// current index of traveled path group
	unsigned int currentPathTravelIndex = 0u;
public:
	StateModel();

	void calcNextState(void);
};

#endif /* STATEMODEL_H */
