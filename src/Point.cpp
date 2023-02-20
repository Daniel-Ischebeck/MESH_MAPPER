class Point {
public:
    // Point(int _index, double _x, double _y, double _z) {        //: index(_index), x(_x), y(_y), z(_z){}
    //     index = _index;
    //     x = _x;
    //     y = _y;
    //     z = _z;
    // }
    Point();
    ~Point();


    //copy constructor 
    Point(const Point &thePoint) {
        index = thePoint.index;
        x = thePoint.x;
        y = thePoint.y;
        z = thePoint.z;
    }
    
    //asssignment
    Point &Point::operator=(const Point &thePoint) {
        if(this==&thePoint) return(*this);

        index = thePoint.index;
        x = thePoint.x;
        y = thePoint.y;
        z = thePoint.z;
        return *this;
    }
    

private:
    int index;
    double x,y,z;
};

Point::Point(int _index, double _x, double _y, double _z) {        //: index(_index), x(_x), y(_y), z(_z){}
        index = _index;
        x = _x;
        y = _y;
        z = _z;
    }
