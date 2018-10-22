/*
 * PositionUpdater.cpp
 *
 *  Created on: 22.10.2018
 *      Author: grauvogs
 */

#include "PositionUpdater.h"

PositionUpdater::PositionUpdater() {
	pthread_create(&positionUpdateThread, NULL, &updatePosition, NULL);
}

PositionUpdater::~PositionUpdater()
{
	pthread_cancel(positionUpdateThread);
}



