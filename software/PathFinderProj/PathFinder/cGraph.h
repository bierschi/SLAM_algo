#ifndef CMAP_H
#define CMAP_H

#include <unordered_map>
#include <vector>
#include <limits>
#include <algorithm>
#include <iostream>
#include <commontypes.h>

using namespace std;


// used dijkstra:
// https://gist.github.com/vertexclique/7410577

// alternative:
// https://github.com/search?q=dijkstra+c%2B%2B


class cGraph
{
    unordered_map<std::string, const unordered_map<std::string, int>> vertices;

public:
    vector<std::string> shortest_path(std::string start, std::string finish);
    vector<std::string> addMap2Graph(map_t &map, int blockSize, std::string ego_pos, Direction direction);

private:
    std::string addPoint2Graph(int **&orig_map, int row, int column, int map_size_x, int map_size_y, int blockSize, Direction egoDirection);
    void add_vertex(std::string name, const unordered_map<std::string, int>& edges);
};

#endif // CMAP_H
