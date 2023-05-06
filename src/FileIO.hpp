#include <iostream>
#include <fstream>
#include <sstream>

#include <string>
#include <vector>

#include <Eigen/Dense> // eigen is header only - link in cmake
#include <Eigen/Sparse>

#include "Point.hpp"
#include "Face.hpp"

bool readFile(std::vector<Point> &listOfPoints,
              std::vector<Face> &listOfFaces,
              std::vector<std::vector<int>> &faces,
              std::vector<std::vector<double>> &points,
              std::string filePath);

bool outputTRIfile(std::vector<Point> &listOfPoints,
                   std::vector<Face> &listOfFaces,
                   std::string filePath);

bool outputUVfile(std::vector<Face> &listOfFaces,
                  Eigen::MatrixXi &faceMatrix,
                  Eigen::VectorXd &u_coords,
                  Eigen::VectorXd &v_coords,
                  std::string filePath,
                  int attributeFlag,
                  std::vector<double> &results);

bool readOBJ(std::vector<Point> &listOfPoints,
             std::vector<Face> &listOfFaces,
             std::string filePath);

void OBJToTRI();

void outputOFFfile(std::vector<Point> &listOfPoints,
                   std::vector<Face> &listOfFaces,
                   std::string filePath);

void OBJtoOFF();
