#ifndef EDGE_HPP
#define EDGE_HPP
class Edge
{
public:
    Edge();
    Edge(int _index1, int _index2);
    ~Edge();

    Edge(const Edge &theEdge); // copy

    Edge &operator=(const Edge &theEdge);

    bool operator==(const Edge &theEdge);

    int get_index1();

private:
    int index1, index2;
};
#endif