#ifndef FACE_HPP
#define FACE_HPP

class Face
{
public:
    Face();

    Face(int _faceIndex, int _aIndex, int _bIndex, int _cIndex);

    ~Face();

    Face(const Face &theFace); // copy

    Face &operator=(const Face &theFace);

private:
    int faceIndex;
    int aIndex, bIndex, cIndex; //triangle with corners defined by verticies  A,B,C
};

#endif