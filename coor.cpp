#include "coor.h"

Coordinate::Coordinate() :
x(0), y(0) {}

Coordinate::Coordinate(double x, double y) :
x(x), y(y) {}

Coordinate Coordinate::operator+(const Coordinate &other)
{
    return (Coordinate(x+other.x, y+other.y));
}

Coordinate Coordinate::operator-(const Coordinate &other)
{
    return (Coordinate(x-other.x, y-other.y));
}