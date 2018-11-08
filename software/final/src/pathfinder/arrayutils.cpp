#include "pathfinder/arrayutils.h"
#include <iostream>
#include <fstream>

//  ############################### EASY HELPER ##############################

/* detect direction, ego vehicle is pointing to */
Direction getDirection(float theta)
{
    if(theta >= 315.0f || theta < 45.0f)
        return NORTH;
    if(theta >= 45.0f  && theta < 135.0f)
        return EAST;
    if(theta >= 135.0f && theta < 225.0f)
        return SOUTH;
    if(theta >= 225.0f && theta < 315.0f)
        return WEST;
    return NONE;
}

bool isEmpty(int value)
{
    if(value == WHITE_VALUE)
    {
        return true;
    }
    return false;
}

//  ############################### ARRAY MODIFICATION ##############################
/* clears all "in-between" gray values out of the map, only GRAY_VALUE, white and gray stays */
void clearMapToBlackWhite(map_t &map)
{
    // 0 = black
    // 255 = white

    for( int i = 1; i<map.size_y-1; i++)
        for(int j = 1; j<map.size_x-1; j++)
        {
            if(map.array[i][j] != WHITE_VALUE && map.array[i][j] != GRAY_VALUE && map.array[i][j] > 60)
                map.array[i][j] = WHITE_VALUE;       // set all values to white
            if(map.array[i][j] != WHITE_VALUE && map.array[i][j] != GRAY_VALUE && map.array[i][j] <= 60)
                map.array[i][j] = BLACK_VALUE;       // set all values to BLACK
        }
}

/* fill map with gray-values decending from black to white (dependent on distance) */
void fillGrayGradient(map_t &map, int blockSize)
{
    int grayValue = BLACK_VALUE;  // begin with black
    int halfBlock;

    if(blockSize >= 3)
        halfBlock = (blockSize-1) / 2;
    else
        halfBlock = 1;

    for(int iteration = 0; iteration < 800; iteration++, grayValue+=5)    // get brighter
        for( int i = 0; i < map.size_y; i++)
            for(int j = 0; j < map.size_x; j++)
            {
                if(map.array[i][j] == grayValue)
                {
                    setBlockToGrayvalue(map, i, j, grayValue + 5, halfBlock);
                }
            }
}

void setBlockToGrayvalue(map_t &map, int i, int j, int grayValue, int halfBlock)
{
    for(int row = -halfBlock; row <= halfBlock; row++)
        for(int column = -halfBlock; column <= halfBlock; column++)
            if ((i+row >= 0 && j+column >= 0)
                 && (i+row < map.size_x && j+column < map.size_y)
                 && (isEmpty(map.array[i+row][j+column])))
            {
                map.array[i+row][j+column] = grayValue;
            }
}

//  ############################### FILE ACCESS ##############################

/* load pgm file, standard format (P2\n x y\n  data) */
void getDataFromFile(map_t &map, std::string const&path)
{
    ifstream f_map(path);

    std::string mapLine;
    std::string mapSizeToken;

    // get map size from file
    std::getline(f_map, mapLine);  // P2 -> not needed
    std::getline(f_map, mapLine);  // x y

    mapSizeToken = mapLine.substr(0, mapLine.find(" "));
    map.size_x = std::atoi(mapSizeToken.c_str());

    mapLine = mapLine.substr(mapLine.find(" ")+1, mapLine.length());
    map.size_y = std::atoi(mapLine.substr(0, mapLine.find(" ")).c_str());

    std::cout<<"Map size: \n x: "<<map.size_x << " y: " << map.size_y << endl;


    // create array of pointers of pointers
    map.array = new int*[map.size_y];
    for(int i = 0; i < map.size_y; ++i)
        map.array[i] = new int[map.size_x];


    // fill array with data points
    int row = 0;
    for (std::string line; std::getline(f_map, line); )
    {
        for(int column = 0; column < map.size_x; column++)
        {
            std::string token = line.substr(0, line.find(" "));
            line = line.substr(line.find(" ")+1, line.length());
            map.array[row][column] = std::atoi(token.c_str());
        }
        row++;
    }

    f_map.close();
}

/* load pgm file, standard format (P2\n x y\n  data) */
void getPositionFromFile(int &x, int &y, float &theta, std::string const&path)
{
    ifstream f_position(path);
    std::string line;

    // get map size from file
    std::getline(f_position, line);  // x
    x = std::atoi(line.c_str());

    std::getline(f_position, line);  // y
    y = std::atoi(line.c_str());

    std::getline(f_position, line);  // theta
    theta = std::atof(line.c_str());

    f_position.close();
}

/* save map to new pgm file */
void saveMapToFile(map_t &map, std::string const&path, bool append)
{
    ofstream f_MapOutput;
    if(append)
        f_MapOutput.open(path, std::ios_base::app);
    else
        f_MapOutput.open(path, std::ios_base::trunc);

    f_MapOutput << "P2\n" << map.size_x << " " << map.size_y << " " << WHITE_VALUE << endl;

    for( int i = 0; i< map.size_y; i++)
    {
        for(int j = 0; j< map.size_x; j++)
        {
            f_MapOutput << map.array[i][j]<< " ";
        }
        f_MapOutput << endl;
    }


    f_MapOutput.close();
}

void savePathToFile(std::string &driveway, std::string const&path)
{
    ofstream f_driveway;

    f_driveway.open(path, std::ios_base::trunc);

    f_driveway << driveway;

    f_driveway.close();
}

void saveEgoPosToFile(std::string &position, std::string const&path)
{
    ofstream f_egoPosOut;

    f_egoPosOut.open(path, std::ios_base::trunc);

    f_egoPosOut << position;

    f_egoPosOut.close();
}
