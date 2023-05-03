#include <iostream>
#include <fstream>
#include <sstream>

#include <string>
#include <vector>

#include <chrono> //for timing execution

#include <Eigen/Dense> // eigen is header only - link in cmake
#include <Eigen/Sparse>

#include <Eigen/IterativeLinearSolvers>
#include <Eigen/Geometry> //for rotation


#include "Point.hpp"
#include "Face.hpp"

bool calcTriangleAreas(std::vector<Point> &listOfPoints,
                       std::vector<Face> &listOfFaces,
                       std::vector<double> &listOfAreas);

Eigen::VectorXi find_triangles(int indexWereAfter,
                               std::vector<Face> &listOfFaces);

bool removeTriangles(std::vector<Point> &listOfPoints,
                     std::vector<Face> &listOfFaces,
                     std::vector<std::vector<int>> &faces,
                     Eigen::MatrixXi &faceMatrix);

bool compare_double(double first, double second, double epsilon);

bool compareEdges(std::vector<Edge> &first, std::vector<Edge> &second);

Edge whichEdgeShared(std::vector<Edge> &first, std::vector<Edge> &second, int &indexOfSharedEdge_second, int &indexOfSharedEdge_first);

bool postProcess(std::vector<double> &outputAreas,
                 std::vector<double> &listOfAreas,
                 std::vector<Point> &outputPoints,
                 std::vector<Face> &listOfFaces,
                 Eigen::VectorXd &u_coords,
                 Eigen::VectorXd &v_coords,
                 Eigen::MatrixXi &faceMatrix,
                 std::vector<Point> &listOfPoints,
                 Eigen::VectorXi &pinnedVerticies,
                 std::vector<double> &results,
                 std::vector< Eigen::MatrixXd > &listOfLocalBasis,
                 int scaleFlag

);

bool rotateModel(std::vector<Point> &listOfPoints,
                 Eigen::MatrixXd &pointMatrix,
                 std::vector<std::vector<double>> &points,
                 char axis);



