#include "Face.hpp"

Face::Face() {}

Face::Face(int _faceIndex, int _aIndex, int _bIndex, int _cIndex)
{ //: index(_index), x(_x), y(_y), z(_z){}
    faceIndex = _faceIndex;
    aIndex = _aIndex;
    bIndex = _bIndex;
    cIndex = _cIndex;

    faceEdges.resize(3);
    faceEdges.at(0) = Edge(_aIndex, _bIndex);
    faceEdges.at(1) = Edge(_bIndex, _cIndex);
    faceEdges.at(2) = Edge(_cIndex, _aIndex);
    /*
    edge0 = Edge(_aIndex, _bIndex);
    edge1 = Edge(_bIndex, _cIndex);
    edge2 = Edge(_cIndex, _aIndex);
    faceEdges.push_back(edge0);
    faceEdges.push_back(edge1);
    faceEdges.push_back(edge2);
    */
}

Face::~Face() {}

// copy constructor
Face::Face(const Face &theFace)
{
    faceIndex = theFace.faceIndex;
    aIndex = theFace.aIndex;
    bIndex = theFace.bIndex;
    cIndex = theFace.cIndex;

    faceEdges = theFace.faceEdges;
}

// asssignment
Face &Face::operator=(const Face &theFace)
{
    if (this == &theFace)
        return (*this);

    faceIndex = theFace.faceIndex;
    aIndex = theFace.aIndex;
    bIndex = theFace.bIndex;
    cIndex = theFace.cIndex;

    faceEdges = theFace.faceEdges;
    return *this;
}

// equivalent
bool Face::operator==(const Face &theFace)
{
    if (faceIndex == theFace.faceIndex && aIndex == theFace.aIndex && bIndex == theFace.bIndex && cIndex == theFace.cIndex)
        return true;
    return false;
}

int Face::get_faceIndex() { return faceIndex; }
int Face::get_aIndex() { return aIndex; }
int Face::get_bIndex() { return bIndex; }
int Face::get_cIndex() { return cIndex; }

void Face::set_faceIndex(int newIndex) { faceIndex = newIndex; }

std::vector<Edge> Face::get_faceEdges() { return faceEdges; }

Edge Face::get_edge0() { return edge0; }
