
#include <iostream>

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


    while ( ros::ok() ) {

        if (sm->getMapInitFlag()) {
            std::cout <<"Map was succesfully initialised!" << std::endl;
            pi->processPath();

            //std::cout << "x_pixel: " << sm->getPixelX() << " y_pixel: " << sm->getPixelY() << " theta: " << sm->theta << std::endl;
            sm->setSaveMap(true);

        }


        sleep(3);
        ros::spinOnce();
    }


    return 0;
}
