#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <vector>
#include "SPILibrary.h"
#include "ComStructure.h"

ComStructureType COM_StructRX;
ComStructureType COM_StructTX;

using namespace std;

class PathGroup
{
public:
    PathGroup()
    {

    }

    void getPathTravels(const char * inputFile) {
        FILE * file;
        file = fopen(inputFile, "r");
        char buffer[100] = {0};
        int scanfError = 0;
        float targetX, targetY, theta;

        while (NULL != fgets(buffer, 30, file)) {
			printf("%s", buffer);
			scanfError = sscanf(buffer, "%f, %f, %f", &targetX, &targetY, &theta);

			if(scanfError <= 0) break;

		}
    }
};

class PathTravel
{
	float targetX, targetY, originX, originY;
public:
	PathTravel() {
        targetX = 0.0f;
        targetY = 0.0f;
        originX = 0.0f;
        originY = 0.0f;
    }
};

int main (int argc, char ** argv)
{
	vector<PathTravel> pathVector;
	PathTravel segment;
	PathGroup group;
	pathVector.push_back(segment);
	group.getPathTravels("/home/stefan/HSPCar/STM32_HSPCar/SLAM_algo/software/STM32_PeripheralController/RPI/SRC/test.txt");

    spiOpen();
    printf("Open SPI Interface...\n");
    spiSend(COM_StructTX, COM_StructRX);
	printf("Transmit Structure to STM32...\n");
    spiClose();
    printf("Close SPI Interface...\n");
}



