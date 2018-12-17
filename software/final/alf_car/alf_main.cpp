
#include <iostream>
#include <unistd.h>

#include "ros/ros.h"
#include "SlamMap.h"
#include "Server.h"
#include "PathfinderInterface.h"


int main(int argc, char** argv) {


    std::cout << "starting alf software" << std::endl;

    ros::init(argc, argv, "Alf");

    SlamMap* sm = new SlamMap("map_hector");

    Server* server = new Server(2501, *sm);

    PathfinderInterface* pi = new PathfinderInterface(*sm);

    bool flag = true;

    while ( ros::ok() ) {

        if (sm->getMapInitFlag() & sm->getPoseInitFlag()) {

            if (flag) {
                std::cout <<"Map and Pose was succesfully initialised!" << std::endl;
                pi->processPath();
                sm->setSaveMap(true);
                flag = false;
            }

            std::cout << "x_pixel: " << sm->getPixelX() << " y_pixel: " << sm->getPixelY() << " theta: " << sm->theta << std::endl;
        }

        sm->createTxtPositionFile();
        usleep(200000);
        ros::spinOnce();
    }


    return 0;
}
