#ifndef POINT_HPP
#define POINT_HPP

class Point
{
public:
    Point();

    Point(int _pointIndex, double _x, double _y, double _z);

    ~Point();

    Point(const Point &thePoint); // copy

    Point &operator=(const Point &thePoint);

    double get_index();
    double get_x();
    double get_y();
    double get_z();

private:
    int pointIndex;
    double x, y, z;
};

#endif