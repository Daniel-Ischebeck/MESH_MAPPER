#include "Point.hpp"

// Point(int _index, double _x, double _y, double _z) {        //: index(_index), x(_x), y(_y), z(_z){}
//     index = _index;
//     x = _x;
//     y = _y;
//     z = _z;
// }
Point::Point() {}

Point::Point(int _pointIndex, double _x, double _y, double _z)
{ //: index(_index), x(_x), y(_y), z(_z){}
    pointIndex = _pointIndex;
    x = _x;
    y = _y;
    z = _z;
}

Point::~Point() {}

// copy constructor
Point::Point(const Point &thePoint)
{
    pointIndex = thePoint.pointIndex;
    x = thePoint.x;
    y = thePoint.y;
    z = thePoint.z;
}

// asssignment
Point &Point::operator=(const Point &thePoint)
{
    if (this == &thePoint)
        return (*this);

    pointIndex = thePoint.pointIndex;
    x = thePoint.x;
    y = thePoint.y;
    z = thePoint.z;
    return *this;
}

double Point::get_index() { return pointIndex; }
double Point::get_x() { return x; }
double Point::get_y() { return y; }
double Point::get_z() { return z; }

void Point::set_index(int newIndex) { pointIndex = newIndex; }
void Point::set_x(double newX) { x = newX; }
void Point::set_y(double newY) { y = newY; }
void Point::set_z(double newZ) { z = newZ; }