#include "cnode.h"

#define NORTH_POS 0
#define WEST_POS 0
#define SOUTH_POS 0
#define EAST_POS 0

cNode::cNode()
{

}

cNode::cNode(int row, int column, int value)
{
    this->row = row;
    this->column = column;
    this->grayValue = value;
}

void cNode::addNeighbors(shared_ptr<cNode> north, shared_ptr<cNode> west, shared_ptr<cNode> south, shared_ptr<cNode> east)
{
    this->neighbor.push_back(north);
    this->neighbor.push_back(west);
    this->neighbor.push_back(south);
    this->neighbor.push_back(east);
}

shared_ptr<cNode> cNode::getNorth()
{
    return this->neighbor[NORTH_POS];
}
shared_ptr<cNode> cNode::getWest()
{
    return this->neighbor[WEST_POS];
}
shared_ptr<cNode> cNode::getSouth()
{
    return this->neighbor[SOUTH_POS];
}
shared_ptr<cNode> cNode::getEast()
{
    return this->neighbor[EAST_POS];
}
