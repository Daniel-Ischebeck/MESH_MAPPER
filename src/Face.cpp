#include "Face.hpp"

Face::Face() {}

Face::Face(int _faceIndex, int _aIndex, int _bIndex, int _cIndex)
{ //: index(_index), x(_x), y(_y), z(_z){}
    faceIndex = _faceIndex;
    aIndex = _aIndex;
    bIndex = _bIndex;
    cIndex = _cIndex;
}

Face::~Face() {}

// copy constructor
Face::Face(const Face &theFace)
{
    faceIndex = theFace.faceIndex;
    aIndex = theFace.aIndex;
    bIndex = theFace.bIndex;
    cIndex = theFace.cIndex;
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
    return *this;
}

int Face::get_faceIndex() { return faceIndex; }
int Face::get_aIndex() { return aIndex; }
int Face::get_bIndex() { return bIndex; }
int Face::get_cIndex() { return cIndex; }

void Face::set_faceIndex(int newIndex) { faceIndex = newIndex; }
