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

class PositionUpdater
{
private:
	pthread_t positionUpdateThread;
	// pointer to static
	static void * updatePosition(void * args) {
		while (1) {
			printf("Update Position here!\n");
			sleep(1u);
		}
	}

public:
	PositionUpdater();
	~PositionUpdater();
};

#endif /* POSITIONUPDATER_H */
