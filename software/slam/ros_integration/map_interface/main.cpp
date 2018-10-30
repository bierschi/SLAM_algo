
#include "ros/ros.h"
#include "SlamMap.h"
#include "Trajectory.h"
#include <tuple>
#include <iostream>

int main(int argc, char**argv){


    ros::init(argc, argv, "SlamMapInterface");
    std::string mapname = "hector";
    int threshold_occupied = 65;
    int threshold_free = 25;

    SlamMap sm(mapname, threshold_occupied, threshold_free);


    Trajectory tr;
    tr.setTrajectoryFlag(false);

    while(ros::ok()) {
        // metadata from slam -> origin position
        std::cout << "origin_x: " << sm.getOriginPosX() << " origin_y: " << sm.getOriginPosY() << std::endl;
        // pose_estimate from slam
        std::cout << "x: " <<sm.pose_x << " y: " << sm.pose_y << " z: " << sm.pose_z << std::endl;
        sleep(1);


        ros::spinOnce();
    }

    return 0;
}