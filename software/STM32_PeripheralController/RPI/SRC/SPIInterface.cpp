#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/types.h>
#include "SPILibrary.h"
#include "Path.h"
#include "PositionUpdater.h"
#include "StateModel.h"

using namespace std;

int main (int argc, char ** argv)
{
	PathGroup group;
	StateModel model;
	group.determinePathTravels("./path.txt");

	printf("Open SPI Interface...\n");
	spiOpen();

	printf("Create State Model...\n");
	model.Main();

	printf("Close SPI Interface...\n");
	spiClose();

    return 0;
}



