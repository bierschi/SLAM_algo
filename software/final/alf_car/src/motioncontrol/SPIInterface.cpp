#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string>
#include <string.h>
#include <sys/types.h>
#include <signal.h>
#include "SPILibrary.h"
#include "Path.h"
#include "PositionUpdater.h"
#include "StateModel.h"

using namespace std;

// signalhandler for termination of program
void sigIntHandler(int signum);
bool exitProgram = false;


// #########################################################################################
// signal handlers
// #########################################################################################

void sigIntHandler(int signum)
{
	exitProgram = true;
}

int main (int argc, char ** argv)
{
	StateModel model;

    signal(SIGINT, sigIntHandler);

	if(argc > 1)
	{
		if(0 == strcmp(argv[1], "-i"))
		{
			model.setScanAtStartup(true);
		}
	}

	printf("Open SPI Interface...\n");
	spiOpen();

	printf("Create State Model...\n");
    model.Init();

    while(exitProgram == false)
	{
        std::string test("192;168\n189;164\n123;84");
        model.updatePosition(0,1, 0.0f);
        model.updatePathTravels(test);
		printf("Calculating Next State...\n");
		model.Main();
	}

    printf("Close Model...\n");
    model.Close();

	printf("Close SPI Interface...\n");
	spiClose();

    return 0;
}



