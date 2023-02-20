#ifndef POINT_HPP
#define POINT_HPP

class Point
{
public:
    Point();

    Point(int _index, double _x, double _y, double _z);

    ~Point();

    Point(const Point &thePoint); // copy

    Point &operator=(const Point &thePoint);

private:
    int index;
    double x, y, z;
};

#endif