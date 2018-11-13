//
// Created by christian on 27.10.18.
//


#include "slam/SlamMap.h"

/**
 * Constructor for a SlamMap instance
 *
 * USAGE:
 *      SlamMap sm;
 *
 * @param mapname: string for the map name, default = "hector"
 * @param threshold_occupied: int number for the occupied threshold
 * @param threshold_free: int number for the free threshold
 */
SlamMap::SlamMap(const std::string& mapname, int threshold_occupied, int threshold_free)
        : mapname_(mapname),
          save_map_(false),
          mapInitData(false),
          threshold_occupied_(threshold_occupied),
          threshold_free_(threshold_free),
          mapCounter(0),
          mapHeight(0),
          mapWidth(0),
          mapResolution(-1.0),
          position_x(0.0),
          position_y(0.0),
          position_z(0.0),
          orientation_w(0.0),
          theta(0.0)
{

    ros::NodeHandle n1, n2, n3, n4;

    ROS_INFO("Waiting for the map!");

    // Subscriber
    map_sub_ = n1.subscribe("map", 2, &SlamMap::mapCallback, this);
    map_metadata_sub_ = n2.subscribe("map_metadata", 2, &SlamMap::mapMetadataCallback, this);
    pose_sub_ = n3.subscribe("slam_out_pose", 2, &SlamMap::poseCallback, this);

    // Publisher
    reset_map_pub_ = n4.advertise<std_msgs::String>("syscommand", 2);

}

/**
 * Destructor in SlamMap
 */
SlamMap::~SlamMap() {

}

/**
 * callback for the "slam_out_pose" topic
 */
void SlamMap::poseCallback(const geometry_msgs::PoseStampedConstPtr &pose) {

    //ROS_INFO("Received SLAM Position Estimate!");

    position_x = pose->pose.position.x;
    position_y = pose->pose.position.y;
    position_z = pose->pose.position.z;
    orientation_w = pose->pose.orientation.w;


    //theta = atan2(position_y, position_x) * 180/PI;

    tf::Quaternion q(pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z, pose->pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw, yaw_degrees;
    m.getRPY(roll, pitch, yaw);
    yaw_degrees = yaw * 180.0 / M_PI;
    //convert negative to positive angles
    if (yaw_degrees < 0)
        yaw_degrees +=360.0;

    theta = yaw_degrees;

    //alternativ: theta = yaw * 180.0 / M_PI;

    //std::cout << "x: " << position_x << " y: " << position_y << " theta: " << theta << " yaw: " << yaw<<std::endl;

}

/**
 * callback for the "map_metadata" topic
 *
 * @param metadata: const nav_msgs::MapMetaDataConstPtr reference to obtain the map metadata
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

    mapWidth = metadata->width;
    mapHeight = metadata->height;
    mapResolution = metadata->resolution;

    mapInitData = true;

}

/**
 * callback for the "map" topic
 *
 * @param map: const nav_msgs::OccupancyGridConstPtr reference to obtain the raw map data
 */
void SlamMap::mapCallback(const nav_msgs::OccupancyGridConstPtr& map) {

    //ROS_INFO("Received a %d X %d map @ %.3f m/pix", map->info.width, map->info.height, map->info.resolution);

    if (save_map_) {
        createPgmMapFile(map);
        createTxtPositionFile();
    }

    mapInterface(map);

}

/**
 * updated the mapData vector periodically
 *
 * @param map: nav_msgs.:OccupancyGridConstPtr&
 */
void SlamMap::mapInterface(const nav_msgs::OccupancyGridConstPtr& map) {

    /*
    for (std::vector<int>::size_type i = 0; i != map->data.size() ; i++) {
        printf("%d ", map->data[i]);
    }*/ //data -1, 0-100

    mapData.resize(map->data.size());

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
 * creates a pgm file to view the slam map
 *
 * @param map: const nav_msgs::OccupancyGridConstPtr reference to obtain the raw map data
 */
void SlamMap::createPgmMapFile(const nav_msgs::OccupancyGridConstPtr& map) {

    std::string mapCounterStr = std::to_string(mapCounter);
    //std::string mapdatafile = mapname_ + mapCounterStr +  ".pgm";
    std::string mapdatafile = mapname_ + ".pgm";
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
 * creates a txt file containing the raw map data
 *
 * @param fileName: string file name for the txt file
 * @param mapData: std::vector<int> containing the map data
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

/**
 * creates a position.txt file containing x, y, theta
 */
void SlamMap::createTxtPositionFile() {

    std::string mapCounterStr = std::to_string(mapCounter);
    //std::string positionDataFile = "position" + mapCounterStr +  ".txt";
    std::string positionDataFile = "position" + ".txt"
    FILE* out = fopen(positionDataFile.c_str(), "w");
    fprintf(out, "%d\n%d\n%.2f", getPixelX(), getPixelY(), theta);
    fclose(out);

}

/**
 *
 */
/*
void SlamMap::sendSlamMap(ServerSocket& sock) {

    std::thread run(&SlamMap::sendSlamMapThread, this, std::ref(sock));
    run.detach();
}
*/
/**
 *
 * @param sock
 */
/*
void SlamMap::sendSlamMapThread(ServerSocket &sock) {
    std::vector<int> v;

    while(true) {
        std::cout << "Test" << std::endl;
        v = getMapData();
        sock.sending(v);
        sleep(2);
    }
}
*/
/**
 * method to reset the Map as a independent thread
 */
void SlamMap::resetMap() {

    std::thread reset(&SlamMap::reset, this);
    reset.detach();

}

/**
 * Thread to reset the Slam Map
 */
void SlamMap::reset() {

    std_msgs::String msg;
    msg.data = "reset";
    reset_map_pub_.publish(msg);

}

/**
 * get the map data
 *
 * @return std::vector<int> mapData
 */
std::vector<int> SlamMap::getMapData() {
    return mapData;
}

/**
 * get the save_map_ flag
 *
 * @return bool save_map_
 */
bool SlamMap::getSaveMap() const {
    return save_map_;
}

/**
 * set the save_map flag
 *
 * @param save_map: bool
 */
void SlamMap::setSaveMap(bool save_map) {
    save_map_ = save_map;
}

/**
 * get the origin x position
 *
 * @return double: origin_pos_x_
 */
double SlamMap::getOriginPosX() const {
    if (mapInitData)
        return origin_pos_x_;
}

/**
 * get the origin y position
 *
 * @return double: origin_pos_y_
 */
double SlamMap::getOriginPosY() const {
    if (mapInitData)
        return origin_pos_y_;
}

/**
 * get the origin z position
 *
 * @return double: origin_pos_z_
 */
double SlamMap::getOriginPosZ() const {
    if (mapInitData)
        return origin_pos_z_;
}

/**
 *  get current x pixel position in map
 *
 * @return pixel_x: int
 */
int SlamMap::getPixelX() {

    if (mapInitData) {
        //int pixel_x = (int)((-getOriginPosX() / mapResolution) - (position_x / mapResolution));
        double pos_tmp = (-getOriginPosX() - position_x);
        int pixel_x = (int) (pos_tmp / mapResolution);
        return pixel_x;
        //double max_breite = mapWidth * mapResolution;
        //double middle = max_breite / 2.0;
        //double act_pos = middle - position_x;
        //int pix = (int)(act_pos / mapResolution);
        //return pixel_x;
        //return pix;
    }
}

/**
 * get current y pixl position in map
 *
 * @return pixel_y: int
 */
int SlamMap::getPixelY() {

    if (mapInitData) {
        //int pixel_y =  (int)((getOriginPosY() / mapResolution) - (position_y / mapResolution));#
        double pos_tmp = (getOriginPosX() - position_y);
        int pixel_y = (int) (pos_tmp / mapResolution);
        return pixel_y;
    }
}