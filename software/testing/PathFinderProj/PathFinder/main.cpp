#include <iostream>
#include <fstream>
#include <cGraph.h>
#include <arrayutils.h>

using namespace std;

static const string INPUT_FOLDER = "../../SLAM_data/";
static const string OUTPUT_FOLDER = "../../PathFinder_data/";

int main()
{
    map_t map;
    cGraph g;
    int blockSize = 5;
    vector<std::string> posDestinations;
    //std::string inputMapName = "map_hector.pgm";
    std::string inputMapName = "map.pgm";
    //std::string mapName = "easy_map.pgm";

    std::cout << endl << "#####################################" << endl;
    std::cout << "### read file..." << endl;
    getDataFromFile(map, INPUT_FOLDER + inputMapName);

    std::cout << endl << "#####################################" << endl;
    std::cout << "### clear map to black, white and gray value..." << endl;
    clearMapToBlackWhite(map);


    std::cout << endl << "#####################################" << endl;
    std::cout << "### save whited map..." << endl;
    saveMapToFile(map, OUTPUT_FOLDER + "map_whited.pgm", false);


    std::cout << endl << "#####################################" << endl;
    std::cout << "### fill map with gradientes and save map..." << endl;
    //fillGrayGradient(map, blockSize);
    saveMapToFile(map, OUTPUT_FOLDER + "map_remarked.pgm", false);



    std::cout << endl << "#####################################" << endl;
    std::cout << "### read position from file..." << endl;
    int x = 0, y = 0;
    int x_native = 0, y_native = 0;
    float theta = 0.0f;
    std::string ego_pos = "406;406";
    getPositionFromFile(x_native, y_native, theta, INPUT_FOLDER + "position.txt");
    // align ego pos to middle of block
    int times = 0;
    int start = 0;


    /*
     * for debugging reason only!!!
     * remove to use data from position.txt
     * */
    ego_pos = "408;393";
    x_native = 408;
    y_native = 393;


    /*
     * map is clustered in blocks, ego position needs to be shifted into middle of block!
     * */
    times = x_native / blockSize;
    start = blockSize / 2 + 1;
    x = times * blockSize + start;

    times = y_native / blockSize;
    start = blockSize / 2 + 1;
    y = times * blockSize + start;

    ego_pos = std::to_string(x) + ";" + std::to_string(y);

    std::cout << "position: " << x << ";" << y << endl;



    //save map in graph
    std::cout << endl << "#####################################" << endl;
    std::cout << "### trigger path calculation..." << endl;
    posDestinations = g.addMap2Graph(map, blockSize, ego_pos, getDirection(theta));
    // possible destinations are transitions from gray nodes to white nodes!




    std::cout << endl << "#####################################" << endl;
    std::cout << "### read map again (to draw path)..." << endl;
    // choose map for vizualization
    getDataFromFile(map, INPUT_FOLDER + "map_whited.pgm");


    bool found = false;
    std::string driveway = "";


    /*
     * search for path to possible destinations (transition from gray to white)
     * if there is no path found from ego position to posDest -> test next posDest
     * reasons for not finding a path could be posDest ist not reachable with white nodes (wall and gray nodes between start end destination
     * or ego is at gray node (no connections to others)
     * */
    for (std::string posDest : posDestinations)
    {
        driveway = "";

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

    saveMapToFile(map, OUTPUT_FOLDER + "map_withPath.pgm", false);
    savePathToFile(driveway, OUTPUT_FOLDER + "driveway.txt");
    std::string egoPosAtMap = std::to_string(x_native) + "\n" + std::to_string(y_native) + "\n" + std::to_string(theta);
    saveEgoPosToFile(egoPosAtMap, OUTPUT_FOLDER + "egoPosAtMap.txt");

    // free disk
    for(int i = 0; i < map.size_x; ++i)
        delete [] map.array[i];
    delete [] map.array;


    return 0;
}
