
#include <iostream>
#include <unistd.h>
#include <signal.h>

#include "ros/ros.h"
#include "SlamMap.h"
#include "Server.h"
#include "PathfinderInterface.h"
#include "StateModel.h"
#include "SPILibrary.h"

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

    printf("Open SPI Interface...\n");
	spiOpen();

    // check if the SPI interface to the STM32 board is opened...
    if(OPEN == spiIsOpen())
    {
        printf("Create State Model...\n");
        spiInterfaceStateModel.Init();
        //################################################
        // scan area 360 degrees:
        spiInterfaceStateModel.setScanAtStartup(true);

        while(ros::ok() && exitProgram == false)
        {
            if (sm->getMapInitFlag() & sm->getPoseInitFlag()) {
                std::cout << "x_pixel: " << sm->getPixelX() << " y_pixel: " << sm->getPixelY() << " theta: " << sm->theta << std::endl;
                spiInterfaceStateModel.updatePosition((float) sm->getPixelX(),(float) sm->getPixelY(), (float) sm->theta);
                spiInterfaceStateModel.Main();

                if(spiInterfaceStateModel.isBusy() == false) break;
                usleep(200000);
            }
            ros::spinOnce();
        }

        // travel along path:
        bool flag = true;

        while ( ros::ok() && exitProgram == false) {

            if (sm->getMapInitFlag() & sm->getPoseInitFlag()) {

                if (flag) {
                    std::cout <<"Map and Pose was succesfully initialised!" << std::endl;
                    pi->processPath();
                    spiInterfaceStateModel.updatePathTravels(pi->drivewayPath);
                    sm->setSaveMap(true);
                    flag = false;
                }
                std::cout << "x_pixel: " << sm->getPixelX() << " y_pixel: " << sm->getPixelY() << " theta: " << sm->theta << std::endl;
            }

            spiInterfaceStateModel.updatePosition((float) sm->getPixelX(),(float) sm->getPixelY(), (float) sm->theta);
            spiInterfaceStateModel.Main();

            // check if the travelled path end was reached, then get new path from PathFinder module (see next cycle)
            if(spiInterfaceStateModel.isBusy() == false)
            {
                flag = true;
            }

            //sm->createTxtPositionFile();
            usleep(200000);
            ros::spinOnce();
        }

        printf("Close Model...\n");
        spiInterfaceStateModel.Close();

        printf("Close SPI Interface...\n");
        spiClose();
    }

    return 0;
}
