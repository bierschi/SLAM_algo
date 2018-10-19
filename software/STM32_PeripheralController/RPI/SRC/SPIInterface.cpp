#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <pthread.h>
#include "SPILibrary.h"
#include "ComStructure.h"

#define MAX_NUM_TRAVELS	1000

ComStructureType COM_StructRX;
ComStructureType COM_StructTX;

using namespace std;

class PathTravel
{
	float targetX, targetY, originX, originY, theta;
public:
	PathTravel(float oX, float oY, float tX, float tY, float th) {
        targetX = tX;
        targetY = tY;
        originX = oX;
        originY = oY;
        theta = th;
    }
};

class PathGroup
{
private:
	// init number of path travels
	PathTravel *travels[MAX_NUM_TRAVELS] = {0x0};
public:
    PathGroup()
    {
    }

    /** determine path travels from file input */
    void determinePathTravels(const char * inputFile) {
    	int counter = 0;
        FILE * file;
        file = fopen(inputFile, "r");
        char buffer[100] = {0};
        int scanfError = 0;
        float originX, originY, targetX, targetY, theta;

        while ((NULL != fgets(buffer, 99, file)) && (counter < MAX_NUM_TRAVELS)) {
			printf("%s", buffer);
			// parse informations from file
			scanfError = sscanf(buffer, "%f, %f, %f, %f, %f", &originX, &originY, &targetX, &targetY, &theta);

			this->travels[counter] = new PathTravel(originX, originY, targetX, targetY, theta);

			if(scanfError <= 0) break;

			counter++;
		}
    }

    /** [DUMMY]  determine path travels from internal sources */
    void determinePathTravels(void)
    {

    }
};

class PositionUpdater
{
private:
	pthread_t positionUpdateThread;
public:
	static void * updatePosition(void * args)
	{
		while (1) {
			printf("Update Position here!\n");
			sleep(1u);
		}
	}

	PositionUpdater()
	{
		pthread_create(&positionUpdateThread, NULL, &updatePosition, NULL);
	}
};

class StateModel
{
	StateModel()
	{

	}
};

int main (int argc, char ** argv)
{
	PathGroup group;
	group.determinePathTravels(".\\path.txt");
	PositionUpdater posUpdater;

    spiOpen();
    printf("Open SPI Interface...\n");
    spiSend(COM_StructTX, COM_StructRX);
	printf("Transmit Structure to STM32...\n");
    spiClose();
    printf("Close SPI Interface...\n");

    while(1);

    return 0;
}



