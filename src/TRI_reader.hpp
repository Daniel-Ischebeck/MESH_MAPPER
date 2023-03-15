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

#include "Point.hpp"
#include "Face.hpp"

void prepMatricies(std::vector<Point> &listOfPoints,
                   std::vector<Face> &listOfFaces,
                   std::vector<double> &listOfAreas,
                   Eigen::VectorXi &pinnedVerticies,
                   Eigen::VectorXd &pinnedUV,
                   Eigen::SparseMatrix<double> &A,
                   Eigen::MatrixXd &RHS);


bool calcTriangleAreas(std::vector<Point> &listOfPoints,
                       std::vector<Face> &listOfFaces,
                       std::vector<double> &listOfAreas);




bool compare_double(double first, double second, double epsilon);