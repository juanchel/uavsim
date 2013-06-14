#ifndef __uavsim__coor__
#define __uavsim__coor__

#include <iostream>

class Coordinate
{
public:
    double x,y;
    
    Coordinate();
    Coordinate(double x, double y);
    Coordinate operator+(const Coordinate& other);
    Coordinate operator-(const Coordinate& other);
};

#endif