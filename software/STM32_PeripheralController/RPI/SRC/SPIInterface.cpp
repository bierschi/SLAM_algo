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

class PathTravel
{
	float targetX, targetY, originX, originY;
    public:
	PathTravel()
    {
    }

    void test(void)
    {
        printf("Hallo Welt!");
    }
};

int main (int argc, char ** argv)
{
	vector<PathTravel> pathVector;
	PathTravel segment;
	pathVector.push_back(segment);
	segment.test();

    spiOpen();
    printf("Open SPI Interface...\n");
    spiSend(COM_StructTX, COM_StructRX);
	printf("Transmit Structure to STM32...\n");
    spiClose();
    printf("Close SPI Interface...\n");
}



