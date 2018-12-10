//
// Created by christian on 06.12.18.
//

#include "PathfinderInterface.h"

PathfinderInterface::PathfinderInterface(SlamMap &sm) : sm_(sm),
                                                        blockSize(5),
                                                        x_native_(0),
                                                        y_native_(0),
                                                        theta_(0.0),
                                                        OUTPUT_FOLDER("../PathFinder_data/")
{

    if (boost::filesystem::create_directory(OUTPUT_FOLDER)) {
        std::cout << "Created directory!" << std::endl;
    }

}

PathfinderInterface::~PathfinderInterface() {

    for(int i = 0; i < map.size_x; ++i)
        delete [] map.array[i];

}


void PathfinderInterface::processPath() {


    initSlamMap();

    calcPath();
}


void PathfinderInterface::initSlamMap() {

    // fetch data from slam map
    map.size_x = sm_.getMapHeight();
    map.size_y = sm_.getMapWidth();
    std::vector<int> mapData = sm_.getMapData();

    std::cout << "sizex: " << map.size_x << std::endl;

    // fill array with current data from slam map
    map.array = new int*[map.size_y];
    for(int i = 0; i < map.size_y; ++i)
        map.array[i] = new int[map.size_x];

    for (int row = 0; row < map.size_y; row++) {
        for (int column = 0; column < map.size_x; column++) {
            int i = column + (map.size_x - row - 1) * map.size_y;
            map.array[row][column] = mapData[i];
        }

    }

}

void PathfinderInterface::calcPath() {

    int x = 0, y = 0;

    x_native_ = sm_.getPixelX();
    y_native_ = sm_.getPixelY();
    theta_    = sm_.theta;


    std::string ego_pos = "406;406";

    // align ego pos to middle of block
    int times = 0;
    int start = 0;


    times = x_native_ / blockSize;
    start = blockSize / 2 + 1;
    x = times * blockSize + start;

    times = y_native_ / blockSize;
    start = blockSize / 2 + 1;
    y = times * blockSize + start;


    ego_pos = std::to_string(x) + ";" + std::to_string(y);

    std::cout << "position: " << x << ";" << y << endl;

    setBlockToGrayvalue(map, x, y, WHITE_VALUE, 4);

    //save map in graph
    std::cout << endl << "#####################################" << endl;
    std::cout << "### trigger path calculation..." << endl;
    posDestinations = g.addMap2Graph(map, blockSize, ego_pos, getDirection(theta_));

    bool found = false;
    std::string driveway = "";

    for (std::string posDest : posDestinations)
    {
        driveway = "";
//        std::string dest_node = "526;502";
//        std::cout << "enter destination node: ";
//        std::cin>>dest_node;

        std::cout << endl << "#####################################" << endl;
        std::cout << "### calculated path " << ego_pos << " -> " << posDest << endl;


        int seq = 0, i = 0, j = 0;
        std::string token, delimiter = ";";

        for (std::string vertex : g.shortest_path(ego_pos, posDest))
        {
            cout << "Solution path hop: " << seq << " Node : " << vertex;
            driveway = vertex + "\n" + driveway ;

            token = vertex.substr(0, vertex.find(delimiter));
            i = std::stoi(token);

            token = vertex.erase(0, vertex.find(delimiter) + delimiter.length());
            j = std::stoi(token);
            cout << " value: " << map.array[i][j] << endl;
            setBlockToGrayvalue(map, i, j, 0, 1);
            seq++;
            found = true;
        }
        driveway = ego_pos + "\n" + driveway ;
        if(found == true)
            break;
    }


    saveMapToFile(map, OUTPUT_FOLDER + "map_withPath.pgm");
    savePathToFile(driveway, OUTPUT_FOLDER + "driveway.txt");
    std::string egoPosAtMap = std::to_string(x_native_) + "\n" + std::to_string(y_native_) + "\n" + std::to_string(theta_);
    saveEgoPosToFile(egoPosAtMap, OUTPUT_FOLDER + "egoPosAtMap.txt");


}

Direction PathfinderInterface::getDirection(float theta)
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

void PathfinderInterface::clearMapToBlackWhite(map_t &map)
{
    // 0 = black
    // 254 = white

    for( int i = 1; i<map.size_y-1; i++)
        for(int j = 1; j<map.size_x-1; j++)
        {
            if(map.array[i][j] != WHITE_VALUE && map.array[i][j] != GRAY_VALUE && map.array[i][j] > 60)
                map.array[i][j] = WHITE_VALUE;       // set all values to white
            if(map.array[i][j] != WHITE_VALUE && map.array[i][j] != GRAY_VALUE && map.array[i][j] <= 60)
                map.array[i][j] = BLACK_VALUE;       // set all values to BLACK
        }

}

void PathfinderInterface::fillGrayGradient(map_t &map)
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

void PathfinderInterface::setBlockToGrayvalue(map_t &map, int i, int j, int grayValue, int halfBlock)
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


void PathfinderInterface::saveMapToFile(map_t &map, std::string fileName)
{

    FILE* out = fopen(fileName.c_str(), "w");
    fprintf(out, "P2\n%d %d 255\n",
            map.size_x,
            map.size_y);


    for( int i = 0; i< map.size_y; i++)
    {
        for(int j = 0; j< map.size_x; j++)
        {
            fprintf(out, "%d ",map.array[i][j]);
        }
        fprintf(out, "\n");
    }

    fclose(out);
}

void PathfinderInterface::savePathToFile(std::string &driveway, std::string const&path)
{
    ofstream f_driveway;

    f_driveway.open(path, std::ios_base::trunc);

    f_driveway << driveway;

    f_driveway.close();
}

void PathfinderInterface::saveEgoPosToFile(std::string &position, std::string const&path)
{
    ofstream f_egoPosOut;

    f_egoPosOut.open(path, std::ios_base::trunc);

    f_egoPosOut << position;

    f_egoPosOut.close();
}