#ifndef ARRAYUTILS_H
#define ARRAYUTILS_H
#include <cGraph.h>
#include <commontypes.h>


//void getDataFromFile(int **&orig_map, int& map_size_x, int& map_size_y, std::string const&path);
void getDataFromFile(map_t &map, std::string const&path);
void getPositionFromFile(int &x, int &y, float &theta, std::string const&path);
void saveMapToFile(map_t &map, std::string const&path, bool append);
void savePathToFile(std::string &driveway, std::string const&path);
void saveEgoPosToFile(std::string &position, std::string const&path);

void clearMapToBlackWhite(map_t &map);
void fillGrayGradient(map_t &map, int blockSize);
void setBlockToGrayvalue(map_t &map, int i, int j, int grayValue, int halfBlock);

bool isEmpty(int value);
Direction getDirection(float theta);

#endif // ARRAYUTILS_H
