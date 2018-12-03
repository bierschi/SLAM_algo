
#include <iostream>

#include "ros/ros.h"
#include "slam/SlamMap.h"
#include "communication/server/Server.h"



int main(int argc, char** argv) {


    std::cout << "starting alf software" << std::endl;

    ros::init(argc, argv, "Alf");

    SlamMap* sm = new SlamMap("map_hector");

    Server* server = new Server(2501, *sm);

    while ( ros::ok() ) {

        std::cout << "x_pixel: " << sm->getPixelX() << " y_pixel: " << sm->getPixelY() << " theta: " << sm->theta << std::endl;
        sm->setSaveMap(true);

        sleep(2);
        ros::spinOnce();
    }


    return 0;
}
