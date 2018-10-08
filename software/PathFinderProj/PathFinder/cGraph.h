#ifndef CMAP_H
#define CMAP_H

#include <unordered_map>
#include <vector>
#include <limits>
#include <algorithm>
#include <iostream>

#include <cnode.h>

using namespace std;

class cGraph
{
    unordered_map<std::string, const unordered_map<std::string, int>> vertices;

public:
    void add_vertex(std::string name, const unordered_map<std::string, int>& edges);
    vector<std::string> shortest_path(std::string start, std::string finish);
    void addMap2Graph(int **&orig_map, int map_size_x, int map_size_y, int blockSize);
    void addPGM(std::string const&path);

    vector<cNode> nodes;
    int size_x;
    int size_y;

private:
    void addPoint2Graph(int **&orig_map, int row, int column, int map_size_x, int map_size_y, int blockSize);
    void fillMap(std::string const&path, int **&orig_map);
    void transferMapToGraph(int **&orig_map);
};

#endif // CMAP_H
