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
	pthread_t positionUpdateThread;
	PositionStructureType position = {0};
	// pointer to static
	void * updatePosition(void * args) {
		struct timespec ts_sleep = {0}, ts_remaining = {0};
		ts_sleep.tv_nsec = 100000000L; // 100 ms delay

		while (1) {
			printf("Update Position...\n");

			nanosleep(&ts_sleep, &ts_remaining);
		}
	}

public:
	PositionUpdater();
	~PositionUpdater();
	PositionStructureType getPosition(void);

};

#endif /* POSITIONUPDATER_H */
