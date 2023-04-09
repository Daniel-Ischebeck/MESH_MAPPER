#ifndef POINT_HPP
#define POINT_HPP

class Point
{
public:
    Point();

    Point(int _pointIndex, double _x, double _y, double _z);

    Point(double _u, double _v);    //for uv coords

    ~Point();

    Point(const Point &thePoint); // copy

    Point &operator=(const Point &thePoint);

    bool operator==(const Point &thePoint);

    double get_index();
    double get_x();
    double get_y();
    double get_z();

    void set_index(int newIndex);
    void set_x(double newX);
    void set_y(double newY);
    void set_z(double newZ);

private:
    int pointIndex;
    double x, y, z;
};

#endif