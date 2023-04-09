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
#include "processing.hpp"

bool prepMatricies(std::vector<Point> &listOfPoints,
                   std::vector<Face> &listOfFaces,
                   std::vector<double> &listOfAreas,
                   Eigen::VectorXi &pinnedVerticies,
                   Eigen::VectorXd &pinnedUV,
                   Eigen::SparseMatrix<double> &A,
                   Eigen::MatrixXd &RHS);



bool prepSolutionOutput(Eigen::VectorXd &u_coords,
                        Eigen::VectorXd &v_coords,
                        Eigen::VectorXi &pinnedVerticies,
                        Eigen::VectorXd &solution,
                        Eigen::VectorXd &pinnedUV,
                        int size_listOfPoints);

