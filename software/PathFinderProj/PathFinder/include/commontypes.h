#ifndef COMMONTYPES_H
#define COMMONTYPES_H

#define GRAY_VALUE 127
//205
#define BLACK_VALUE 0
#define WHITE_VALUE 255

struct map_t
{
  int size_x;
  int size_y;
  int **array;
};


enum Direction
{
    NORTH,
    EAST,
    SOUTH,
    WEST,
    NONE,
};



#endif // COMMONTYPES_H
