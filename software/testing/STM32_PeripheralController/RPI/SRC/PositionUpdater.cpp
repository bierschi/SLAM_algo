/*
 * PositionUpdater.cpp
 *
 *  Created on: 22.10.2018
 *      Author: grauvogs
 */

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <fcntl.h>
#include "PositionUpdater.h"
#include "StateModel.h"

PositionUpdater::PositionUpdater() {
}

PositionUpdater::~PositionUpdater()
{
}

PositionStructureType PositionUpdater::getPosition(void)
{
	return position;
}

void PositionUpdater::updatePosition(void)
{
	FILE * file = NULL;
	file = fopen(POSITION_FILE, "r");
	char buffer[100] = {0};

	if(file != NULL)
	{
		int num;
		// parse position information from lines of file

		// get x position:
		fgets(buffer, 99, file);
		num = sscanf(buffer, "%f", &position.x);

		// get y position:
		fgets(buffer, 99, file);
		num = sscanf(buffer, "%f", &position.y);

		// get theta of position:
		fgets(buffer, 99, file);
		num = sscanf(buffer, "%f", &position.theta);
        
	}
	else
	{
		printf("PositionUpdater: Error opening file input!\n");
	}
}
