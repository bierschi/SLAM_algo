
#include <iostream>

#include "ros/ros.h"
#include "slam/SlamMap.h"


int main(int argc, char** argv) {


    std::cout << "starting alf software" << std::endl;
    ros::init(argc, argv, "Alf");

    SlamMap sm("hector");


    while ( ros::ok() ) {

        std::cout << "x_pixel: " << sm.getPixelX() << " y_pixel: " << sm.getPixelY() << " theta: " << sm.theta << std::endl;
        sm.setSaveMap(true);

        sleep(3);
        ros::spinOnce();
    }


    return 0;
}