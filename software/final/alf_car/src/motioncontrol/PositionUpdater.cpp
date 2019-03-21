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
	ifstream f_position(POSITION_FILE);
	
	if(false == f_position.is_open()) cout << "Error reading file!" << endl;

    std::string line;

    // get map size from file
    std::getline(f_position, line);  // x
    position.x = std::atoi(line.c_str());

    std::getline(f_position, line);  // y
    position.y = 200 - std::atoi(line.c_str());

    std::getline(f_position, line);  // theta
    position.theta = std::atof(line.c_str());

    f_position.close();
	
}

void PositionUpdater::updatePosition(float xpos, float ypos, float theta)
{
	position.x = xpos;
	position.y = 200 - ypos;
	position.theta = theta;
}
