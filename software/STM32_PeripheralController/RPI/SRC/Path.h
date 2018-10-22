/*
 * Path.h
 *
 *  Created on: 22.10.2018
 *      Author: grauvogs
 */

#ifndef PATH_H
#define PATH_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#define MAX_NUM_PATH_TRAVELS	1000

class PathTravel {
private:
	float targetX, targetY, originX, originY, theta;
	bool travelled;
public:
	PathTravel(float oX, float oY, float tX, float tY, float th);
};

class PathGroup {
private:
	// init number of path travels
	PathTravel *travels[MAX_NUM_PATH_TRAVELS] = { 0x0 };
	// creation timestamp of path
	time_t creationTime = 0;
	bool pathChanged = true;

public:
	PathGroup();

	/** determine path travels from file input */
	void determinePathTravels(const char * inputFile);

	/** [DUMMY]  determine path travels from internal sources */
	void determinePathTravels(void);
	void cleanPathTravels(void);
	bool getPathChanged(void);

};

#endif /* PATH_H */
