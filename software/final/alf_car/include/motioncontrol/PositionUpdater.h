/*
 * PositionUpdater.h
 *
 *  Created on: 22.10.2018
 *      Author: grauvogs
 */

#ifndef POSITIONUPDATER_H
#define POSITIONUPDATER_H

#include <stdio.h>
#include <unistd.h>
#include <pthread.h>

typedef struct PositionStructure
{
	float x;
	float y;
	float theta;
} PositionStructureType;

class PositionUpdater
{
private:
	PositionStructureType position = {0};

public:
	PositionUpdater();
	~PositionUpdater();
	PositionStructureType getPosition(void);
	void updatePosition(void);

};

#endif /* POSITIONUPDATER_H */
