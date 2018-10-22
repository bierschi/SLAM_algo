#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/types.h>
#include "SPILibrary.h"
#include "ComStructure.h"
#include "Path.h"
#include "PositionUpdater.h"
#include "StateModel.h"

// communication structures for STM32 peripheral
ComStructureType COM_StructRX;
ComStructureType COM_StructTX;

using namespace std;

int main (int argc, char ** argv)
{
	PathGroup group;
	group.determinePathTravels("./path.txt");
	PositionUpdater posUpdater;

	group.cleanPathTravels();

    spiOpen();
    printf("Open SPI Interface...\n");
    spiSend(COM_StructTX, COM_StructRX);
	printf("Transmit Structure to STM32...\n");
    spiClose();
    printf("Close SPI Interface...\n");

    while(1);

    return 0;
}



