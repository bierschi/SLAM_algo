//
// Created by christian on 27.10.18.
//

#include "slam/SlamMap.h"

/**
 *
 * @param mapname
 * @param threshold_occupied
 * @param threshold_free
 */
SlamMap::SlamMap(const std::string& mapname, int threshold_occupied, int threshold_free)
        : mapname_(mapname),
          save_map_(false),
          threshold_occupied_(threshold_occupied),
          threshold_free_(threshold_free),
          mapCounter(0),
          mapHeight(0),
          mapWidth(0),
          position_x(0.0),
          position_y(0.0),
          position_z(0.0),
          orientation_w(0.0),
          theta(0.0)
{

    ros::NodeHandle n1, n2, n3;
    ROS_INFO("Waiting for the map!");
    map_sub_ = n1.subscribe("map", 2, &SlamMap::mapCallback, this);
    map_metadata_sub_ = n2.subscribe("map_metadata", 2, &SlamMap::mapMetadataCallback, this);
    pose_sub_ = n3.subscribe("slam_out_pose", 2, &SlamMap::poseCallback, this);

}

/**
 *
 */
SlamMap::~SlamMap() {

}

/**
 *
 */
void SlamMap::poseCallback(const geometry_msgs::PoseStampedConstPtr &pose) {

    //ROS_INFO("Received SLAM Position Estimate!");

    position_x = pose->pose.position.x;
    position_y = pose->pose.position.y;
    position_z = pose->pose.position.z;
    orientation_w = pose->pose.orientation.w;

    theta = atan2(position_y, position_x) * 180/PI;

    //tf::Quaternion q(pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z, pose->pose.orientation.w);
    //tf::Matrix3x3 m(q);
    //double roll, pitch, yaw;
    //m.getRPY(roll, pitch, yaw);
    //std::cout << "r: " << roll << " p: " << pitch << " y: " << yaw << std::endl;

    //std::cout << "x: " << position_x << " y: " << position_y << " theta: " << theta << " yaw: " << yaw<<std::endl;

}

/**
 *
 * @param metadata
 */
void SlamMap::mapMetadataCallback(const nav_msgs::MapMetaDataConstPtr &metadata) {

    ROS_INFO("Received MapMetadata Origin Position!");

    origin_pos_x_ = metadata->origin.position.x;
    origin_pos_y_ = metadata->origin.position.y;
    origin_pos_z_ = metadata->origin.position.z;

    origin_or_x_ = metadata->origin.orientation.x;
    origin_or_y_ = metadata->origin.orientation.y;
    origin_or_z_ = metadata->origin.orientation.z;
    origin_or_w_ = metadata->origin.orientation.w;

}

/**
 *
 * @param map
 */
void SlamMap::mapCallback(const nav_msgs::OccupancyGridConstPtr& map) {

    //ROS_INFO("Received a %d X %d map @ %.3f m/pix", map->info.width, map->info.height, map->info.resolution);

    if (save_map_)
        createPgmMapFile(map);

    mapInterface(map);

}

/**
 *
 * @param map
 */
void SlamMap::mapInterface(const nav_msgs::OccupancyGridConstPtr& map) {

    /*
    for (std::vector<int>::size_type i = 0; i != map->data.size() ; i++) {
        printf("%d ", map->data[i]);
    }*/ //data -1, 0-100

    mapData.resize(map->data.size());
    mapHeight = map->info.height;
    mapWidth = map->info.width;

    for (unsigned int y = 0; y < map->info.height; y++) {
        for (unsigned int x = 0; x < map->info.width; x++) {
            unsigned int i = x + (map->info.height - y - 1) * map->info.width;

            if (map->data[i] >= 0 && map->data[i] <= threshold_free_) {

                //fputc(254, out);
                mapData[i] = 254;


            } else if (map->data[i] >= threshold_occupied_) {

                //fputc(000, out);
                mapData[i] = 0;

            } else {

                //fputc(205, out);
                mapData[i] = 205;

            }
        }
    }

    /*
    FILE* out = fopen("intarray_SlamMap.txt", "w");
    for(int s = 0; s < mapData.size(); s++) {
        fprintf(out, "%d ", mapData[s]);

        if (s && s%map->info.width == 0) {

           fprintf(out, "\n");

        }
    }*/

}

/**
 *
 * @param map
 */
void SlamMap::createPgmMapFile(const nav_msgs::OccupancyGridConstPtr& map) {

    std::string mapCounterStr = std::to_string(mapCounter);
    std::string mapdatafile = mapname_ + mapCounterStr +  ".pgm";

    ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());

    FILE* out = fopen(mapdatafile.c_str(), "w");

    if (!out) {

        ROS_ERROR("Could not save map file to %s", mapdatafile.c_str());
        return;

    }

    fprintf(out, "P2\n%d %d 255\n",
            map->info.width,
            map->info.height);
    for(unsigned int y = 0; y < map->info.height; y++) {
        for(unsigned int x = 0; x < map->info.width; x++) {

            unsigned int i = x + (map->info.height -y -1) * map->info.width;

            if (map->data[i] >= 0 && map->data[i] <= threshold_free_) {

                //fputc(254, out);
                fprintf(out, "%d ",254);

            } else if (map->data[i] >= threshold_occupied_) {

                //fputc(000, out);
                fprintf(out, "%d ",0);

            } else {

                //fputc(205, out);
                fprintf(out, "%d ",205);

            }
        }
        fprintf(out, "\n");
    }

    fclose(out);

    ROS_INFO("DONE\n");
    mapCounter++;
    setSaveMap(false);

}

/**
 *
 * @param mapData
 */
void SlamMap::createTxtMapFile(std::string fileName, std::vector<int> mapData) {

    FILE* out = fopen(fileName.c_str(), "w");
    for(int s = 0; s < mapData.size(); s++) {
        fprintf(out, "%d ", mapData[s]);

        if (s && s%mapWidth == 0) {

            fprintf(out, "\n");

        }
    }
    fclose(out);
}

void SlamMap::createTxtPositionFile() {

    FILE* out = fopen("position.txt", "w");
    fprintf(out, "%d\n%d\n%.2f", (int)position_x, (int)position_y, theta);
    fclose(out);

}

std::vector<int> SlamMap::getMapData() {
    return mapData;
}

/**
 *
 * @return
 */
bool SlamMap::getSaveMap() const {
    return save_map_;
}

/**
 *
 * @param save_map
 */
void SlamMap::setSaveMap(bool save_map) {
    save_map_ = save_map;
}

/**
 *
 * @return
 */
double SlamMap::getOriginPosX() const {
    return origin_pos_x_;
}

/**
 *
 * @return
 */
double SlamMap::getOriginPosY() const {
    return origin_pos_y_;
}

/**
 *
 * @return
 */
double SlamMap::getOriginPosZ() const {
    return origin_pos_z_;
}
