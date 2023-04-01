#ifndef FACE_HPP
#define FACE_HPP

#include <vector>
#include "Edge.hpp"

class Face
{
public:
    Face();

    Face(int _faceIndex, int _aIndex, int _bIndex, int _cIndex);

    ~Face();

    Face(const Face &theFace); // copy

    Face &operator=(const Face &theFace);

    bool operator==(const Face &theFace);

    int get_faceIndex();
    int get_aIndex();
    int get_bIndex();
    int get_cIndex();

    void set_faceIndex(int newIndex);
    // adding this set functon as when preprocessing models still want consistent index after removing some triangles


    std::vector<Edge> get_faceEdges();
    Edge get_edge0();

    void set_winding(int _windingDirection);
    int get_winding();

private:
    int faceIndex;
    int aIndex, bIndex, cIndex; // triangle with corners defined by verticies  A,B,C
    Edge  edge0, edge1, edge2;
    std::vector<Edge> faceEdges;
    int windingDirection=0;
};



#endif