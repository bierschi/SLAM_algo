//
// Created by christian on 27.10.18.
//

#ifndef MAP_INTERFACE_SLAMMAP_H
#define MAP_INTERFACE_SLAMMAP_H

#include <string>

#include "ros/ros.h"
#include "nav_msgs/GetMap.h"
#include "geometry_msgs/PoseStamped.h"


class SlamMap {

private:
    std::string mapname_;

    ros::Subscriber map_sub_;
    ros::Subscriber map_metadata_sub_;
    ros::Subscriber pose_sub_;

    bool save_map_;
    int threshold_occupied_, threshold_free_;
    int mapCounter;

    double origin_pos_x_, origin_pos_y_, origin_pos_z_;
    double origin_or_x_, origin_or_y_, origin_or_z_, origin_or_w_;

    //double pose_x, pose_y, pose_z;

public:
    SlamMap(const std::string& mapname, int threshold_occupied, int threshold_free);
    ~SlamMap();

    double pose_x, pose_y, pose_z;

    void mapCallback(const nav_msgs::OccupancyGridConstPtr& map);
    void mapMetadataCallback(const nav_msgs::MapMetaDataConstPtr& metadata);
    void poseCallback(const geometry_msgs::PoseStampedConstPtr& pose);

    void createMapFile(const nav_msgs::OccupancyGridConstPtr& map);
    void mapInterface(const nav_msgs::OccupancyGridConstPtr& map);

    bool getSaveMap() const;
    void setSaveMap(bool);

    double getOriginPosX() const;
    double getOriginPosY() const;
    double getOriginPosZ() const;

};



#endif //MAP_INTERFACE_SLAMMAP_H
