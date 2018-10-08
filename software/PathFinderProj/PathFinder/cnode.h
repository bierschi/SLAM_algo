#ifndef CNODE_H
#define CNODE_H

#include <vector>
#include <memory>

using namespace std;

class cNode
{
public:
    cNode();
    cNode(int row, int column, int value);
    int grayValue;
    int row;
    int column;
    vector<shared_ptr<cNode>> neighbor;
    void addNeighbors(shared_ptr<cNode> north, shared_ptr<cNode> west, shared_ptr<cNode> south, shared_ptr<cNode> east);
    shared_ptr<cNode> getNorth();
    shared_ptr<cNode> getWest();
    shared_ptr<cNode> getSouth();
    shared_ptr<cNode> getEast();
};

#endif // CNODE_H
