
#include "ros/ros.h"
#include "SlamMap.h"


int main(int argc, char**argv){


    ros::init(argc, argv, "SlamMapInterface");
    std::string mapname = "hector";
    int threshold_occupied = 65;
    int threshold_free = 25;

    SlamMap sm(mapname, threshold_occupied, threshold_free);

    //ros::spin();
    while(ros::ok()) {
        ros::spinOnce();
    }

    return 0;
}