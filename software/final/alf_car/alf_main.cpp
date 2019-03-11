
#include <iostream>
#include <unistd.h>
#include <signal.h>

#include "ros/ros.h"
#include "SlamMap.h"
#include "Server.h"
#include "PathfinderInterface.h"
#include "StateModel.h"

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


int main(int argc, char** argv) {


    std::cout << "starting alf software" << std::endl;

    signal(SIGINT, sigIntHandler);

    ros::init(argc, argv, "Alf");

    SlamMap* sm = new SlamMap("map_hector");

    Server* server = new Server(2501, *sm);

    PathfinderInterface* pi = new PathfinderInterface(*sm);

    //################################################
    // Create StateModel to control vehicle (SPIInterface to STM32)
    StateModel spiInterfaceStateModel;

    spiInterfaceStateModel.setScanAtStartup(true);

    printf("Open SPI Interface...\n");
	spiOpen();

	printf("Create State Model...\n");
    spiInterfaceStateModel.Init();
    //################################################

    bool flag = true;

    while ( ros::ok() && exitProgram == false) {

        if (sm->getMapInitFlag() & sm->getPoseInitFlag()) {

            if (flag) {
                std::cout <<"Map and Pose was succesfully initialised!" << std::endl;
                pi->processPath();
                spiInterfaceStateModel.updatePathTravels(test);
                sm->setSaveMap(true);
                flag = false;
            }

            std::cout << "x_pixel: " << sm->getPixelX() << " y_pixel: " << sm->getPixelY() << " theta: " << sm->theta << std::endl;
        }

        spiInterfaceStateModel.updatePosition(sm->getPixelX(),sm->getPixelY(), sm->theta);
        spiInterfaceStateModel.Main();

        //sm->createTxtPositionFile();
        //usleep(200000);
        ros::spinOnce();
    }

    printf("Close Model...\n");
    model.Close();

    printf("Close SPI Interface...\n");
    spiClose();

    return 0;
}
