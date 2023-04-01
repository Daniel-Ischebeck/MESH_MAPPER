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

#include <igl/boundary_loop.h>
#include <igl/boundary_facets.h>


#include "Point.hpp"
#include "Face.hpp"

bool prepMatricies(std::vector<Point> &listOfPoints,
                   std::vector<Face> &listOfFaces,
                   std::vector<double> &listOfAreas,
                   Eigen::VectorXi &pinnedVerticies,
                   Eigen::VectorXd &pinnedUV,
                   Eigen::SparseMatrix<double> &A,
                   Eigen::MatrixXd &RHS);

bool calcTriangleAreas(std::vector<Point> &listOfPoints,
                       std::vector<Face> &listOfFaces,
                       std::vector<double> &listOfAreas);

bool rotateModel(std::vector<Point> &listOfPoints,
                 Eigen::MatrixXd &pointMatrix,
                 std::vector<std::vector<double>> &points,
                 char axis);

bool removeTriangles(std::vector<Point> &listOfPoints,
                     std::vector<Face> &listOfFaces,
                     std::vector<std::vector<int>> &faces,
                     Eigen::MatrixXi &faceMatrix);

bool compare_double(double first, double second, double epsilon);

bool prepSolutionOutput(Eigen::VectorXd &u_coords,
                        Eigen::VectorXd &v_coords,
                        Eigen::VectorXi &pinnedVerticies,
                        Eigen::VectorXd &solution,
                        Eigen::VectorXd &pinnedUV,
                        int size_listOfPoints);

Eigen::VectorXi find_triangles(int indexWereAfter,
                               std::vector<Face> &listOfFaces);

bool compareEdges(std::vector<Edge> &first, std::vector<Edge> &second);

Edge whichEdgeShared(std::vector<Edge> &first, std::vector<Edge> &second, int &indexOfSharedEdge_second, int &indexOfSharedEdge_first);