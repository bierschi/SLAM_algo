//
// Created by christian on 06.12.18.
//

#ifndef ALF_PATHFINDERINTERFACE_H
#define ALF_PATHFINDERINTERFACE_H

#include "SlamMap.h"
#include "cGraph.h"
#include "arrayutils.h"


class PathfinderInterface {

private:
    int blockSize;
    int x_native_, y_native_;
    float theta_;
    vector<std::string> posDestinations;

    map_t map;
    cGraph g;
    SlamMap& sm_;

public:
    PathfinderInterface(SlamMap& sm);
    ~PathfinderInterface();

    void processPath();
    void calcPath();
    void clearMapToBlackWhite(map_t& map);
    void fillGrayGradient(map_t& map);
    void setBlockToGrayvalue(map_t& map, int i, int j, int grayValue, int halfBlock);
    void saveMapToFile(map_t& m);

};

#endif //ALF_PATHFINDERINTERFACE_H
