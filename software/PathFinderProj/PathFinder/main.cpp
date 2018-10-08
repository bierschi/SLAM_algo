#include <iostream>
#include <fstream>
#include <cGraph.h>
#include <cnode.h>

// used dijkstra:
// https://gist.github.com/vertexclique/7410577

// alternative:
// https://github.com/search?q=dijkstra+c%2B%2B

using namespace std;

static const string APP_FOLDER = "../data/";
static int map_size_x, map_size_y;
static int **orig_map;

void getDataFromFile(int **&orig_map, int& map_size_x, int& map_size_y, std::string const&path);
void clearMapToBlackWhite();
void fillGrayGradient(int blockSize);
void saveMapToFile(int **&orig_map, int map_size_x, int map_size_y, std::string const&path, bool append);
bool isEmpty(int value);
void setBlockToGrayvalue(int i, int j, int grayValue, int halfBlock);

int main()
{
    cGraph g;
    int blockSize = 6;

    getDataFromFile(orig_map, map_size_x, map_size_y, APP_FOLDER + "map.pgm");
    //getDataFromFile(orig_map, APP_FOLDER + "easy_map.pgm");

    clearMapToBlackWhite();

    saveMapToFile(orig_map, map_size_x, map_size_y, APP_FOLDER + "map_whited.pgm", false);

    fillGrayGradient(blockSize);

    saveMapToFile(orig_map, map_size_x, map_size_y, APP_FOLDER + "map_remarked.pgm", false);


    // blocksize 6
    std::string ego_pos = "406;406";
    std::string dest_node = "526;502";
    int seq = 0, i = 0, j = 0;
    std::string token, delimiter = ";";

    //save map in graph
    g.addMap2Graph(orig_map, map_size_x, map_size_y, blockSize);

    // choose map for vizualization
    getDataFromFile(orig_map, map_size_x, map_size_y, APP_FOLDER + "map.pgm");

    for (std::string vertex : g.shortest_path(ego_pos, dest_node))
    {
        cout << "Solution path hop: " << seq << " Node : " << vertex;

        token = vertex.substr(0, vertex.find(delimiter));
        i = std::stoi(token);

        token = vertex.erase(0, vertex.find(delimiter) + delimiter.length());
        j = std::stoi(token);
        cout << " value: " << orig_map[i][j] << endl;
        setBlockToGrayvalue(i, j, 0, 1);
        seq++;
    }

    saveMapToFile(orig_map, map_size_x, map_size_y, APP_FOLDER + "map_withPath.pgm", false);

    // free disk
    for(int i = 0; i < map_size_x; ++i)
        delete [] orig_map[i];
    delete [] orig_map;


    return 0;
}

/* clears all "in-between" gray values out of the map, only 127, white and gray stays */
void clearMapToBlackWhite()
{
    // 0 = black
    // 255 = white

    for( int i = 1; i<map_size_y-1; i++)
        for(int j = 1; j<map_size_x-1; j++)
        {
            if(orig_map[i][j] != 255 && orig_map[i][j] != 127 && orig_map[i][j] > 60)
                orig_map[i][j] = 255;       // set all values to white
            if(orig_map[i][j] != 255 && orig_map[i][j] != 127 && orig_map[i][j] <= 60)
                orig_map[i][j] = 0;       // set all values to white
        }
}

/* fill map with gray-values decending from black to white (dependent on distance) */
void fillGrayGradient(int blockSize)
{
    int grayValue = 0;  // begin with black
    int halfBlock;

    if(blockSize >= 3)
        halfBlock = (blockSize-1) / 2;
    else
        halfBlock = 1;

    for(int iteration = 0; iteration < 800; iteration++, grayValue+=5)    // get brighter
        for( int i = 0; i < map_size_y; i++)
            for(int j = 0; j < map_size_x; j++)
            {
                if(orig_map[i][j] == grayValue)
                {
                    setBlockToGrayvalue(i, j, grayValue + 5, halfBlock);
                }
            }
}

void setBlockToGrayvalue(int i, int j, int grayValue, int halfBlock)
{
    for(int row = -halfBlock; row <= halfBlock; row++)
        for(int column = -halfBlock; column <= halfBlock; column++)
            if ((i+row >= 0 && j+column >= 0)
                 && (i+row < map_size_x && j+column < map_size_y)
                 && (isEmpty(orig_map[i+row][j+column])))
            {
                orig_map[i+row][j+column] = grayValue;
            }
}

bool isEmpty(int value)
{
    if(value == 255)
    {
        return true;
    }
    return false;
}

/* load pgm file, standard format (P2\n x y\n  data) */
void getDataFromFile(int **&orig_map, int& map_size_x, int& map_size_y, std::string const&path)
{
    ifstream f_lidarOutput(path);

    std::string mapLine;
    std::string mapSizeToken;

    // get map size from file
    std::getline(f_lidarOutput, mapLine);  // P2 -> not needed
    std::getline(f_lidarOutput, mapLine);  // x y

    mapSizeToken = mapLine.substr(0, mapLine.find(" "));
    map_size_x = std::atoi(mapSizeToken.c_str());

    mapLine = mapLine.substr(mapLine.find(" ")+1, mapLine.length());
    map_size_y = std::atoi(mapLine.substr(0, mapLine.find(" ")).c_str());

    std::cout<<"Map size: \n x: "<<map_size_x << " y: " << map_size_y << endl;


    // create array of pointers of pointers
    orig_map = new int*[map_size_y];
    for(int i = 0; i < map_size_y; ++i)
        orig_map[i] = new int[map_size_x];


    // fill array with data points
    int row = 0;
    for (std::string line; std::getline(f_lidarOutput, line); )
    {
        for(int column = 0; column < map_size_x; column++)
        {
            std::string token = line.substr(0, line.find(" "));
            line = line.substr(line.find(" ")+1, line.length());
            orig_map[row][column] = std::atoi(token.c_str());
        }
        row++;
    }

    f_lidarOutput.close();
}


/* save map to new pgm file */
void saveMapToFile(int **&orig_map, int map_size_x, int map_size_y, std::string const&path, bool append)
{
    ofstream f_MapOutput;
    if(append)
        f_MapOutput.open(path, std::ios_base::app);
    else
        f_MapOutput.open(path, std::ios_base::trunc);

    f_MapOutput << "P2\n" << map_size_x << " " << map_size_y << " 255" << endl;

    for( int i = 0; i< map_size_y; i++)
    {
        for(int j = 0; j< map_size_x; j++)
        {
            f_MapOutput << orig_map[i][j]<< " ";
        }
        f_MapOutput << endl;
    }


    f_MapOutput.close();
}
