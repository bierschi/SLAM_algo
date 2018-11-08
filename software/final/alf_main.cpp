
#include <iostream>

#include "ros/ros.h"
#include "slam/SlamMap.h"


int main(int argc, char** argv) {


    std::cout << "starting alf software" << std::endl;
    ros::init(argc, argv, "Alf");

    SlamMap sm("hector");


    while ( ros::ok() ) {


        ros::spinOnce();
    }


    return 0;
}