#include "Edge.hpp"

Edge::Edge() {}

Edge::Edge(int _index1, int _index2)
{
    index1 = _index1;
    index2 = _index2;
}

bool Edge::operator==(const Edge &theEdge)
{
    // two edges are equal if index1=index1 and index2=index2     or   backwards?    index1=index2 and index2=index1
    if ((index1 == theEdge.index1 && index2 == theEdge.index2) || (index1 == theEdge.index2 && index2 == theEdge.index1))
        return true;
    return false;
}

Edge::~Edge() {}

// copy constructor
Edge::Edge(const Edge &theEdge)
{
    index1 = theEdge.index1;
    index2 = theEdge.index2;
}

// asssignment
Edge &Edge::operator=(const Edge &theEdge)
{
    if (this == &theEdge)
        return (*this);

    index1 = theEdge.index1;
    index2 = theEdge.index2;

    return *this;
}

int Edge::get_index1() { return index1; }
