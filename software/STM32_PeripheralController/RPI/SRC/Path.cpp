/*
 * Path.cpp
 *
 *  Created on: 22.10.2018
 *      Author: grauvogs
 */

#include <Path.h>

PathTravel::PathTravel(float oX, float oY, float tX, float tY, float th) {
        targetX = tX;
        targetY = tY;
        originX = oX;
        originY = oY;
        theta = th;
        travelled = false;
}

PathGroup::PathGroup()
{
}

/** determine path travels from file input */
void PathGroup::determinePathTravels(const char * inputFile) {
	int counter = 0;
    FILE * file;
    file = fopen(inputFile, "r");
    char buffer[100] = {0};
    int scanfError = 0;
    float originX, originY, targetX, targetY, theta;

    while ((NULL != file) && (NULL != fgets(buffer, 99, file)) && (counter < MAX_NUM_PATH_TRAVELS)) {
		printf("%s", buffer);
		// parse informations from file
		scanfError = sscanf(buffer, "%f, %f, %f, %f, %f", &originX, &originY, &targetX, &targetY, &theta);

		this->travels[counter] = new PathTravel(originX, originY, targetX, targetY, theta);

		if(scanfError <= 0) break;

		counter++;
	}

    if(NULL != file)
    {
    	fclose(file);
    }
}

/** [DUMMY]  determine path travels from internal sources */
void PathGroup::determinePathTravels(void) {

}

void PathGroup::cleanPathTravels(void) {
	for (int i = 0; i < MAX_NUM_PATH_TRAVELS; i++) {
		if (NULL != travels[i])
			delete travels[i];
	}
}

